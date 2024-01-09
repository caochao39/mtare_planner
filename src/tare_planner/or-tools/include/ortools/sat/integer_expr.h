// Copyright 2010-2018 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OR_TOOLS_SAT_INTEGER_EXPR_H_
#define OR_TOOLS_SAT_INTEGER_EXPR_H_

#include <functional>
#include <vector>

#include "ortools/base/int_type.h"
#include "ortools/base/integral_types.h"
#include "ortools/base/logging.h"
#include "ortools/base/macros.h"
#include "ortools/sat/integer.h"
#include "ortools/sat/model.h"
#include "ortools/sat/precedences.h"
#include "ortools/sat/sat_base.h"
#include "ortools/sat/sat_solver.h"

namespace operations_research {
namespace sat {

// A really basic implementation of an upper-bounded sum of integer variables.
// The complexity is in O(num_variables) at each propagation.
//
// Note that we assume that there can be NO integer overflow. This must be
// checked at model validation time before this is even created.
//
// TODO(user): If one has many such constraint, it will be more efficient to
// propagate all of them at once rather than doing it one at the time.
//
// TODO(user): Explore tree structure to get a log(n) complexity.
//
// TODO(user): When the variables are Boolean, use directly the pseudo-Boolean
// constraint implementation. But we do need support for enforcement literals
// there.
class IntegerSumLE : public PropagatorInterface {
 public:
  // If refied_literal is kNoLiteralIndex then this is a normal constraint,
  // otherwise we enforce the implication refied_literal => constraint is true.
  // Note that we don't do the reverse implication here, it is usually done by
  // another IntegerSumLE constraint on the negated variables.
  IntegerSumLE(const std::vector<Literal>& enforcement_literals,
               const std::vector<IntegerVariable>& vars,
               const std::vector<IntegerValue>& coeffs,
               IntegerValue upper_bound, Model* model);

  // We propagate:
  // - If the sum of the individual lower-bound is > upper_bound, we fail.
  // - For all i, upper-bound of i
  //      <= upper_bound - Sum {individual lower-bound excluding i).
  bool Propagate() final;
  void RegisterWith(GenericLiteralWatcher* watcher);

 private:
  // Fills integer_reason_ with all the current lower_bounds. The real
  // explanation may require removing one of them, but as an optimization, we
  // always keep all the IntegerLiteral in integer_reason_, and swap them as
  // needed just before pushing something.
  void FillIntegerReason();

  const std::vector<Literal> enforcement_literals_;
  const IntegerValue upper_bound_;

  Trail* trail_;
  IntegerTrail* integer_trail_;
  RevIntegerValueRepository* rev_integer_value_repository_;

  // Reversible sum of the lower bound of the fixed variables.
  bool is_registered_ = false;
  IntegerValue rev_lb_fixed_vars_;

  // Reversible number of fixed variables.
  int rev_num_fixed_vars_;

  // Those vectors are shuffled during search to ensure that the variables
  // (resp. coefficients) contained in the range [0, rev_num_fixed_vars_) of
  // vars_ (resp. coeffs_) are fixed (resp. belong to fixed variables).
  std::vector<IntegerVariable> vars_;
  std::vector<IntegerValue> coeffs_;
  std::vector<IntegerValue> max_variations_;

  std::vector<Literal> literal_reason_;

  // Parallel vectors.
  std::vector<IntegerLiteral> integer_reason_;
  std::vector<IntegerValue> reason_coeffs_;

  DISALLOW_COPY_AND_ASSIGN(IntegerSumLE);
};

// A min (resp max) contraint of the form min == MIN(vars) can be decomposed
// into two inequalities:
//   1/ min <= MIN(vars), which is the same as for all v in vars, "min <= v".
//      This can be taken care of by the LowerOrEqual(min, v) constraint.
//   2/ min >= MIN(vars).
//
// And in turn, 2/ can be decomposed in:
//   a) lb(min) >= lb(MIN(vars)) = MIN(lb(var));
//   b) ub(min) >= ub(MIN(vars)) and we can't propagate anything here unless
//      there is just one possible variable 'v' that can be the min:
//         for all u != v, lb(u) > ub(min);
//      In this case, ub(min) >= ub(v).
//
// This constraint take care of a) and b). That is:
// - If the min of the lower bound of the vars increase, then the lower bound of
//   the min_var will be >= to it.
// - If there is only one candidate for the min, then if the ub(min) decrease,
//   the ub of the only candidate will be <= to it.
//
// Complexity: This is a basic implementation in O(num_vars) on each call to
// Propagate(), which will happen each time one or more variables in vars_
// changed.
//
// TODO(user): Implement a more efficient algorithm when the need arise.
class MinPropagator : public PropagatorInterface {
 public:
  MinPropagator(const std::vector<IntegerVariable>& vars,
                IntegerVariable min_var, IntegerTrail* integer_trail);

  bool Propagate() final;
  void RegisterWith(GenericLiteralWatcher* watcher);

 private:
  const std::vector<IntegerVariable> vars_;
  const IntegerVariable min_var_;
  IntegerTrail* integer_trail_;

  std::vector<IntegerLiteral> integer_reason_;

  DISALLOW_COPY_AND_ASSIGN(MinPropagator);
};

// An abs constraint of the form target = Abs(var).
//   1/ target <= MAX(var, -var).
//   2/ abs_var >= 0 if lb(var) < 0 and ub(var) > 0, (vars).
//      abs_var >= lb(var) if lb(var) >= 0
//      abs_var >= -ub(var) if ub(var) <= 0
class AbsPropagator : public PropagatorInterface {
 public:
  AbsPropagator(const IntegerVariable var, IntegerVariable abs_var,
                IntegerTrail* integer_trail);

  bool Propagate() final;
  void RegisterWith(GenericLiteralWatcher* watcher);

 private:
  const IntegerVariable var_;
  const IntegerVariable abs_var_;
  IntegerTrail* integer_trail_;

  std::vector<IntegerLiteral> integer_reason_;

  DISALLOW_COPY_AND_ASSIGN(AbsPropagator);
};

// Propagates a * b = c. Basic version, we don't extract any special cases, and
// we only propagates the bounds.
//
// TODO(user): For now this only works on variables that are non-negative.
// TODO(user): Deal with overflow.
class PositiveProductPropagator : public PropagatorInterface {
 public:
  PositiveProductPropagator(IntegerVariable a, IntegerVariable b,
                            IntegerVariable p, IntegerTrail* integer_trail);

  bool Propagate() final;
  void RegisterWith(GenericLiteralWatcher* watcher);

 private:
  const IntegerVariable a_;
  const IntegerVariable b_;
  const IntegerVariable p_;
  IntegerTrail* integer_trail_;

  DISALLOW_COPY_AND_ASSIGN(PositiveProductPropagator);
};

// Propagates a / b = c. Basic version, we don't extract any special cases, and
// we only propagates the bounds.
//
// TODO(user): For now this only works on variables that are non-negative.
// TODO(user): This only propagate the direction => c, do the reverse.
// TODO(user): Deal with overflow.
// TODO(user): Unit-test this like the ProductPropagator.
class DivisionPropagator : public PropagatorInterface {
 public:
  DivisionPropagator(IntegerVariable a, IntegerVariable b, IntegerVariable c,
                     IntegerTrail* integer_trail);

  bool Propagate() final;
  void RegisterWith(GenericLiteralWatcher* watcher);

 private:
  const IntegerVariable a_;
  const IntegerVariable b_;
  const IntegerVariable c_;
  IntegerTrail* integer_trail_;

  DISALLOW_COPY_AND_ASSIGN(DivisionPropagator);
};

// Propagates var_a / cst_b = var_c. Basic version, we don't extract any special
// cases, and we only propagates the bounds. cst_b must be > 0.
class FixedDivisionPropagator : public PropagatorInterface {
 public:
  FixedDivisionPropagator(IntegerVariable a, IntegerValue b, IntegerVariable c,
                          IntegerTrail* integer_trail);

  bool Propagate() final;
  void RegisterWith(GenericLiteralWatcher* watcher);

 private:
  const IntegerVariable a_;
  const IntegerValue b_;
  const IntegerVariable c_;
  IntegerTrail* integer_trail_;

  DISALLOW_COPY_AND_ASSIGN(FixedDivisionPropagator);
};

// Propagates x * x = s.
// TODO(user): Only works for x nonnegative.
class SquarePropagator : public PropagatorInterface {
 public:
  SquarePropagator(IntegerVariable x, IntegerVariable s,
                   IntegerTrail* integer_trail);

  bool Propagate() final;
  void RegisterWith(GenericLiteralWatcher* watcher);

 private:
  const IntegerVariable x_;
  const IntegerVariable s_;
  IntegerTrail* integer_trail_;

  DISALLOW_COPY_AND_ASSIGN(SquarePropagator);
};

// =============================================================================
// Model based functions.
// =============================================================================

// Weighted sum <= constant.
template <typename VectorInt>
inline std::function<void(Model*)> WeightedSumLowerOrEqual(
    const std::vector<IntegerVariable>& vars, const VectorInt& coefficients,
    int64 upper_bound) {
  // Special cases.
  CHECK_GE(vars.size(), 1);
  if (vars.size() == 1) {
    const int64 c = coefficients[0];
    CHECK_NE(c, 0);
    if (c > 0) {
      return LowerOrEqual(vars[0], upper_bound / c);
    } else {
      const int64 ceil_c = (upper_bound + c + 1) / c;
      return GreaterOrEqual(vars[0], ceil_c);
    }
  }
  if (vars.size() == 2 && (coefficients[0] == 1 || coefficients[0] == -1) &&
      (coefficients[1] == 1 || coefficients[1] == -1)) {
    return Sum2LowerOrEqual(
        coefficients[0] == 1 ? vars[0] : NegationOf(vars[0]),
        coefficients[1] == 1 ? vars[1] : NegationOf(vars[1]), upper_bound);
  }
  if (vars.size() == 3 && (coefficients[0] == 1 || coefficients[0] == -1) &&
      (coefficients[1] == 1 || coefficients[1] == -1) &&
      (coefficients[2] == 1 || coefficients[2] == -1)) {
    return Sum3LowerOrEqual(
        coefficients[0] == 1 ? vars[0] : NegationOf(vars[0]),
        coefficients[1] == 1 ? vars[1] : NegationOf(vars[1]),
        coefficients[2] == 1 ? vars[2] : NegationOf(vars[2]), upper_bound);
  }

  return [=](Model* model) {
    // We split large constraints into a square root number of parts.
    // This is to avoid a bad complexity while propagating them since our
    // algorithm is not in O(num_changes).
    //
    // TODO(user): Alternatively, we could use a O(num_changes) propagation (a
    // bit tricky to implement), or a decomposition into a tree with more than
    // one level. Both requires experimentations.
    //
    // TODO(user): If the initial constraint was an equalilty we will create
    // the "intermediate" variable twice where we could have use the same for
    // both direction. Improve?
    const int num_vars = vars.size();
    if (num_vars > 100) {
      std::vector<IntegerVariable> bucket_sum_vars;

      std::vector<IntegerVariable> local_vars;
      std::vector<IntegerValue> local_coeffs;

      int i = 0;
      const int num_buckets = static_cast<int>(std::round(std::sqrt(num_vars)));
      for (int b = 0; b < num_buckets; ++b) {
        local_vars.clear();
        local_coeffs.clear();
        int64 bucket_lb = 0;
        int64 bucket_ub = 0;
        const int limit = num_vars * (b + 1);
        for (; i * num_buckets < limit; ++i) {
          local_vars.push_back(vars[i]);
          local_coeffs.push_back(IntegerValue(coefficients[i]));
          const int64 term1 = model->Get(LowerBound(vars[i])) * coefficients[i];
          const int64 term2 = model->Get(UpperBound(vars[i])) * coefficients[i];
          bucket_lb += std::min(term1, term2);
          bucket_ub += std::max(term1, term2);
        }

        const IntegerVariable bucket_sum =
            model->Add(NewIntegerVariable(bucket_lb, bucket_ub));
        bucket_sum_vars.push_back(bucket_sum);
        local_vars.push_back(bucket_sum);
        local_coeffs.push_back(IntegerValue(-1));
        IntegerSumLE* constraint = new IntegerSumLE(
            {}, local_vars, local_coeffs, IntegerValue(0), model);
        constraint->RegisterWith(model->GetOrCreate<GenericLiteralWatcher>());
        model->TakeOwnership(constraint);
      }

      // Create the root-level sum.
      local_vars.clear();
      local_coeffs.clear();
      for (const IntegerVariable var : bucket_sum_vars) {
        local_vars.push_back(var);
        local_coeffs.push_back(IntegerValue(1));
      }
      IntegerSumLE* constraint = new IntegerSumLE(
          {}, local_vars, local_coeffs, IntegerValue(upper_bound), model);
      constraint->RegisterWith(model->GetOrCreate<GenericLiteralWatcher>());
      model->TakeOwnership(constraint);
      return;
    }

    IntegerSumLE* constraint = new IntegerSumLE(
        {}, vars,
        std::vector<IntegerValue>(coefficients.begin(), coefficients.end()),
        IntegerValue(upper_bound), model);
    constraint->RegisterWith(model->GetOrCreate<GenericLiteralWatcher>());
    model->TakeOwnership(constraint);
  };
}

// Weighted sum >= constant.
template <typename VectorInt>
inline std::function<void(Model*)> WeightedSumGreaterOrEqual(
    const std::vector<IntegerVariable>& vars, const VectorInt& coefficients,
    int64 lower_bound) {
  // We just negate everything and use an <= constraints.
  std::vector<int64> negated_coeffs(coefficients.begin(), coefficients.end());
  for (int64& ref : negated_coeffs) ref = -ref;
  return WeightedSumLowerOrEqual(vars, negated_coeffs, -lower_bound);
}

// Weighted sum == constant.
template <typename VectorInt>
inline std::function<void(Model*)> FixedWeightedSum(
    const std::vector<IntegerVariable>& vars, const VectorInt& coefficients,
    int64 value) {
  return [=](Model* model) {
    model->Add(WeightedSumGreaterOrEqual(vars, coefficients, value));
    model->Add(WeightedSumLowerOrEqual(vars, coefficients, value));
  };
}

// enforcement_literals => sum <= upper_bound
template <typename VectorInt>
inline std::function<void(Model*)> ConditionalWeightedSumLowerOrEqual(
    const std::vector<Literal>& enforcement_literals,
    const std::vector<IntegerVariable>& vars, const VectorInt& coefficients,
    int64 upper_bound) {
  // Special cases.
  CHECK_GE(vars.size(), 1);
  if (vars.size() == 1) {
    CHECK_NE(coefficients[0], 0);
    if (coefficients[0] > 0) {
      return Implication(
          enforcement_literals,
          IntegerLiteral::LowerOrEqual(
              vars[0], IntegerValue(upper_bound / coefficients[0])));
    } else {
      return Implication(
          enforcement_literals,
          IntegerLiteral::GreaterOrEqual(
              vars[0], IntegerValue(upper_bound / coefficients[0])));
    }
  }
  if (vars.size() == 2 && (coefficients[0] == 1 || coefficients[0] == -1) &&
      (coefficients[1] == 1 || coefficients[1] == -1)) {
    return ConditionalSum2LowerOrEqual(
        coefficients[0] == 1 ? vars[0] : NegationOf(vars[0]),
        coefficients[1] == 1 ? vars[1] : NegationOf(vars[1]), upper_bound,
        enforcement_literals);
  }
  if (vars.size() == 3 && (coefficients[0] == 1 || coefficients[0] == -1) &&
      (coefficients[1] == 1 || coefficients[1] == -1) &&
      (coefficients[2] == 1 || coefficients[2] == -1)) {
    return ConditionalSum3LowerOrEqual(
        coefficients[0] == 1 ? vars[0] : NegationOf(vars[0]),
        coefficients[1] == 1 ? vars[1] : NegationOf(vars[1]),
        coefficients[2] == 1 ? vars[2] : NegationOf(vars[2]), upper_bound,
        enforcement_literals);
  }
  return [=](Model* model) {
    IntegerSumLE* constraint = new IntegerSumLE(
        enforcement_literals, vars,
        std::vector<IntegerValue>(coefficients.begin(), coefficients.end()),
        IntegerValue(upper_bound), model);
    constraint->RegisterWith(model->GetOrCreate<GenericLiteralWatcher>());
    model->TakeOwnership(constraint);
  };
}

// enforcement_literals => sum >= lower_bound
template <typename VectorInt>
inline std::function<void(Model*)> ConditionalWeightedSumGreaterOrEqual(
    const std::vector<Literal>& enforcement_literals,
    const std::vector<IntegerVariable>& vars, const VectorInt& coefficients,
    int64 lower_bound) {
  // We just negate everything and use an <= constraint.
  std::vector<int64> negated_coeffs(coefficients.begin(), coefficients.end());
  for (int64& ref : negated_coeffs) ref = -ref;
  return ConditionalWeightedSumLowerOrEqual(enforcement_literals, vars,
                                            negated_coeffs, -lower_bound);
}

// Weighted sum <= constant reified.
template <typename VectorInt>
inline std::function<void(Model*)> WeightedSumLowerOrEqualReif(
    Literal is_le, const std::vector<IntegerVariable>& vars,
    const VectorInt& coefficients, int64 upper_bound) {
  return [=](Model* model) {
    model->Add(ConditionalWeightedSumLowerOrEqual({is_le}, vars, coefficients,
                                                  upper_bound));
    model->Add(ConditionalWeightedSumGreaterOrEqual(
        {is_le.Negated()}, vars, coefficients, upper_bound + 1));
  };
}

// Weighted sum >= constant reified.
template <typename VectorInt>
inline std::function<void(Model*)> WeightedSumGreaterOrEqualReif(
    Literal is_ge, const std::vector<IntegerVariable>& vars,
    const VectorInt& coefficients, int64 lower_bound) {
  return [=](Model* model) {
    model->Add(ConditionalWeightedSumGreaterOrEqual({is_ge}, vars, coefficients,
                                                    lower_bound));
    model->Add(ConditionalWeightedSumLowerOrEqual(
        {is_ge.Negated()}, vars, coefficients, lower_bound - 1));
  };
}

// Weighted sum == constant reified.
// TODO(user): Simplify if the constant is at the edge of the possible values.
template <typename VectorInt>
inline std::function<void(Model*)> FixedWeightedSumReif(
    Literal is_eq, const std::vector<IntegerVariable>& vars,
    const VectorInt& coefficients, int64 value) {
  return [=](Model* model) {
    // We creates two extra Boolean variables in this case. The alternative is
    // to code a custom propagator for the direction equality => reified.
    const Literal is_le = Literal(model->Add(NewBooleanVariable()), true);
    const Literal is_ge = Literal(model->Add(NewBooleanVariable()), true);
    model->Add(ReifiedBoolAnd({is_le, is_ge}, is_eq));
    model->Add(WeightedSumLowerOrEqualReif(is_le, vars, coefficients, value));
    model->Add(WeightedSumGreaterOrEqualReif(is_ge, vars, coefficients, value));
  };
}

// Weighted sum != constant.
// TODO(user): Simplify if the constant is at the edge of the possible values.
template <typename VectorInt>
inline std::function<void(Model*)> WeightedSumNotEqual(
    const std::vector<IntegerVariable>& vars, const VectorInt& coefficients,
    int64 value) {
  return [=](Model* model) {
    // Exactly one of these alternative must be true.
    const Literal is_lt = Literal(model->Add(NewBooleanVariable()), true);
    const Literal is_gt = is_lt.Negated();
    model->Add(ConditionalWeightedSumLowerOrEqual(is_lt, vars, coefficients,
                                                  value - 1));
    model->Add(ConditionalWeightedSumGreaterOrEqual(is_gt, vars, coefficients,
                                                    value + 1));
  };
}

// Model-based function to create an IntegerVariable that corresponds to the
// given weighted sum of other IntegerVariables.
//
// Note that this is templated so that it can seamlessly accept std::vector<int>
// or std::vector<int64>.
//
// TODO(user): invert the coefficients/vars arguments.
template <typename VectorInt>
inline std::function<IntegerVariable(Model*)> NewWeightedSum(
    const VectorInt& coefficients, const std::vector<IntegerVariable>& vars) {
  return [=](Model* model) {
    std::vector<IntegerVariable> new_vars = vars;
    // To avoid overflow in the FixedWeightedSum() constraint, we need to
    // compute the basic bounds on the sum.
    //
    // TODO(user): deal with overflow here too!
    int64 sum_lb(0);
    int64 sum_ub(0);
    for (int i = 0; i < new_vars.size(); ++i) {
      if (coefficients[i] > 0) {
        sum_lb += coefficients[i] * model->Get(LowerBound(new_vars[i]));
        sum_ub += coefficients[i] * model->Get(UpperBound(new_vars[i]));
      } else {
        sum_lb += coefficients[i] * model->Get(UpperBound(new_vars[i]));
        sum_ub += coefficients[i] * model->Get(LowerBound(new_vars[i]));
      }
    }

    const IntegerVariable sum = model->Add(NewIntegerVariable(sum_lb, sum_ub));
    new_vars.push_back(sum);
    std::vector<int64> new_coeffs(coefficients.begin(), coefficients.end());
    new_coeffs.push_back(-1);
    model->Add(FixedWeightedSum(new_vars, new_coeffs, 0));
    return sum;
  };
}

// Expresses the fact that an existing integer variable is equal to the minimum
// of other integer variables.
inline std::function<void(Model*)> IsEqualToMinOf(
    IntegerVariable min_var, const std::vector<IntegerVariable>& vars) {
  return [=](Model* model) {
    for (const IntegerVariable& var : vars) {
      model->Add(LowerOrEqual(min_var, var));
    }

    MinPropagator* constraint =
        new MinPropagator(vars, min_var, model->GetOrCreate<IntegerTrail>());
    constraint->RegisterWith(model->GetOrCreate<GenericLiteralWatcher>());
    model->TakeOwnership(constraint);
  };
}

// Expresses the fact that an existing integer variable is equal to the maximum
// of other integer variables.
inline std::function<void(Model*)> IsEqualToMaxOf(
    IntegerVariable max_var, const std::vector<IntegerVariable>& vars) {
  return [=](Model* model) {
    std::vector<IntegerVariable> negated_vars;
    for (const IntegerVariable& var : vars) {
      negated_vars.push_back(NegationOf(var));
      model->Add(GreaterOrEqual(max_var, var));
    }

    MinPropagator* constraint = new MinPropagator(
        negated_vars, NegationOf(max_var), model->GetOrCreate<IntegerTrail>());
    constraint->RegisterWith(model->GetOrCreate<GenericLiteralWatcher>());
    model->TakeOwnership(constraint);
  };
}

// Expresses the fact that an existing integer variable is equal to the absolute
// value of another integer variable: abs_var == abs(var).
inline std::function<void(Model*)> IsEqualToAbsOf(IntegerVariable abs_var,
                                                  IntegerVariable var) {
  return [=](Model* model) {
    AbsPropagator* constraint =
        new AbsPropagator(var, abs_var, model->GetOrCreate<IntegerTrail>());
    constraint->RegisterWith(model->GetOrCreate<GenericLiteralWatcher>());
    model->TakeOwnership(constraint);
  };
}

// Creates an integer variable equal to the minimum of other integer variables.
inline std::function<IntegerVariable(Model*)> NewMin(
    const std::vector<IntegerVariable>& vars) {
  return [=](Model* model) {
    IntegerTrail* integer_trail = model->GetOrCreate<IntegerTrail>();

    // The trival bounds will be propagated correctly at level zero.
    IntegerVariable min_var = integer_trail->AddIntegerVariable();
    model->Add(IsEqualToMinOf(min_var, vars));
    return min_var;
  };
}

// Creates an IntegerVariable equal to the maximum of a set of IntegerVariables.
inline std::function<IntegerVariable(Model*)> NewMax(
    const std::vector<IntegerVariable>& vars) {
  return [=](Model* model) {
    IntegerTrail* integer_trail = model->GetOrCreate<IntegerTrail>();

    // The trival bounds will be propagated correctly at level zero.
    IntegerVariable max_var = integer_trail->AddIntegerVariable();
    model->Add(IsEqualToMaxOf(max_var, vars));
    return max_var;
  };
}

// Expresses the fact that an existing integer variable is equal to one of
// the given values, each selected by a given literal.
std::function<void(Model*)> IsOneOf(IntegerVariable var,
                                    const std::vector<Literal>& selectors,
                                    const std::vector<IntegerValue>& values);

template <class T>
void RegisterAndTransferOwnership(Model* model, T* ct) {
  ct->RegisterWith(model->GetOrCreate<GenericLiteralWatcher>());
  model->TakeOwnership(ct);
}
// Adds the constraint: a * b = p.
inline std::function<void(Model*)> ProductConstraint(IntegerVariable a,
                                                     IntegerVariable b,
                                                     IntegerVariable p) {
  return [=](Model* model) {
    IntegerTrail* integer_trail = model->GetOrCreate<IntegerTrail>();
    if (a == b) {
      if (model->Get(LowerBound(a)) >= 0) {
        RegisterAndTransferOwnership(model,
                                     new SquarePropagator(a, p, integer_trail));
      } else if (model->Get(UpperBound(a)) <= 0) {
        RegisterAndTransferOwnership(
            model, new SquarePropagator(NegationOf(a), p, integer_trail));
      } else {
        LOG(FATAL) << "Not supported";
      }
    } else if (model->Get(LowerBound(a)) >= 0 &&
               model->Get(LowerBound(b)) >= 0) {
      RegisterAndTransferOwnership(
          model, new PositiveProductPropagator(a, b, p, integer_trail));
    } else if (model->Get(LowerBound(a)) >= 0 &&
               model->Get(UpperBound(b)) <= 0) {
      RegisterAndTransferOwnership(
          model, new PositiveProductPropagator(a, NegationOf(b), NegationOf(p),
                                               integer_trail));
    } else if (model->Get(UpperBound(a)) <= 0 &&
               model->Get(LowerBound(b)) >= 0) {
      RegisterAndTransferOwnership(
          model, new PositiveProductPropagator(NegationOf(a), b, NegationOf(p),
                                               integer_trail));
    } else if (model->Get(UpperBound(a)) <= 0 &&
               model->Get(UpperBound(b)) <= 0) {
      RegisterAndTransferOwnership(
          model, new PositiveProductPropagator(NegationOf(a), NegationOf(b), p,
                                               integer_trail));
    } else {
      LOG(FATAL) << "Not supported";
    }
  };
}

// Adds the constraint: a / b = d.
inline std::function<void(Model*)> DivisionConstraint(IntegerVariable a,
                                                      IntegerVariable b,
                                                      IntegerVariable c) {
  return [=](Model* model) {
    IntegerTrail* integer_trail = model->GetOrCreate<IntegerTrail>();
    DivisionPropagator* constraint =
        new DivisionPropagator(a, b, c, integer_trail);
    constraint->RegisterWith(model->GetOrCreate<GenericLiteralWatcher>());
    model->TakeOwnership(constraint);
  };
}

// Adds the constraint: a / b = d where b is a constant.
inline std::function<void(Model*)> FixedDivisionConstraint(IntegerVariable a,
                                                           IntegerValue b,
                                                           IntegerVariable c) {
  return [=](Model* model) {
    IntegerTrail* integer_trail = model->GetOrCreate<IntegerTrail>();
    FixedDivisionPropagator* constraint =
        b > 0
            ? new FixedDivisionPropagator(a, b, c, integer_trail)
            : new FixedDivisionPropagator(NegationOf(a), -b, c, integer_trail);
    constraint->RegisterWith(model->GetOrCreate<GenericLiteralWatcher>());
    model->TakeOwnership(constraint);
  };
}

}  // namespace sat
}  // namespace operations_research

#endif  // OR_TOOLS_SAT_INTEGER_EXPR_H_
