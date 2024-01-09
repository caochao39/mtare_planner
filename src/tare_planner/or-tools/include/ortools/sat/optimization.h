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

// Optimization algorithms to solve a LinearBooleanProblem by using the SAT
// solver as a black-box.
//
// TODO(user): Currently, only the MINIMIZATION problem type is supported.

#ifndef OR_TOOLS_SAT_OPTIMIZATION_H_
#define OR_TOOLS_SAT_OPTIMIZATION_H_

#include <functional>
#include <vector>

#include "ortools/sat/boolean_problem.pb.h"
#include "ortools/sat/integer.h"
#include "ortools/sat/integer_search.h"
#include "ortools/sat/model.h"
#include "ortools/sat/sat_base.h"
#include "ortools/sat/sat_solver.h"

namespace operations_research {
namespace sat {

// Tries to minimize the given UNSAT core with a really simple heuristic.
// The idea is to remove literals that are consequences of others in the core.
// We already know that in the initial order, no literal is propagated by the
// one before it, so we just look for propagation in the reverse order.
//
// Important: The given SatSolver must be the one that just produced the given
// core.
void MinimizeCore(SatSolver* solver, std::vector<Literal>* core);

// Like MinimizeCore() with a slower but strictly better heuristic. This
// algorithm should produce a minimal core with respect to propagation. We put
// each literal of the initial core "last" at least once, so if such literal can
// be inferred by propagation by any subset of the other literal, it will be
// removed.
//
// Note that this function doest NOT preserve the order of Literal in the core.
void MinimizeCoreWithPropagation(SatSolver* solver, std::vector<Literal>* core);

// Because the Solve*() functions below are also used in scripts that requires a
// special output format, we use this to tell them whether or not to use the
// default logging framework or simply stdout. Most users should just use
// DEFAULT_LOG.
enum LogBehavior { DEFAULT_LOG, STDOUT_LOG };

// All the Solve*() functions below reuse the SatSolver::Status with a slightly
// different meaning:
// - FEASIBLE: The problem has been solved to optimality.
// - INFEASIBLE: Same meaning, the decision version is already unsat.
// - LIMIT_REACHED: we may have some feasible solution (if solution is
//   non-empty), but the optimality is not proven.

// Implements the "Fu & Malik" algorithm described in:
// Zhaohui Fu, Sharad Malik, "On solving the Partial MAX-SAT problem", 2006,
// International Conference on Theory and Applications of Satisfiability
// Testing. (SAT’06), LNCS 4121.
//
// This algorithm requires all the objective weights to be the same (CHECKed)
// and currently only works on minimization problems. The problem is assumed to
// be already loaded into the given solver.
//
// TODO(user): double-check the correctness if the objective coefficients are
// negative.
SatSolver::Status SolveWithFuMalik(LogBehavior log,
                                   const LinearBooleanProblem& problem,
                                   SatSolver* solver,
                                   std::vector<bool>* solution);

// The WPM1 algorithm is a generalization of the Fu & Malik algorithm to
// weighted problems. Note that if all objective weights are the same, this is
// almost the same as SolveWithFuMalik() but the encoding of the constraints is
// slightly different.
//
// Ansotegui, C., Bonet, M.L., Levy, J.: Solving (weighted) partial MaxSAT
// through satisﬁability testing. In: Proc. of the 12th Int. Conf. on Theory and
// Applications of Satisﬁability Testing (SAT’09). pp. 427-440 (2009)
SatSolver::Status SolveWithWPM1(LogBehavior log,
                                const LinearBooleanProblem& problem,
                                SatSolver* solver, std::vector<bool>* solution);

// Solves num_times the decision version of the given problem with different
// random parameters. Keep the best solution (regarding the objective) and
// returns it in solution. The problem is assumed to be already loaded into the
// given solver.
SatSolver::Status SolveWithRandomParameters(LogBehavior log,
                                            const LinearBooleanProblem& problem,
                                            int num_times, SatSolver* solver,
                                            std::vector<bool>* solution);

// Starts by solving the decision version of the given LinearBooleanProblem and
// then simply add a constraint to find a lower objective that the current best
// solution and repeat until the problem becomes unsat.
//
// The problem is assumed to be already loaded into the given solver. If
// solution is initially a feasible solution, the search will starts from there.
// solution will be updated with the best solution found so far.
SatSolver::Status SolveWithLinearScan(LogBehavior log,
                                      const LinearBooleanProblem& problem,
                                      SatSolver* solver,
                                      std::vector<bool>* solution);

// Similar algorithm as the one used by qmaxsat, this is a linear scan with the
// at-most k constraint encoded in SAT. This only works on problems with
// constant weights.
SatSolver::Status SolveWithCardinalityEncoding(
    LogBehavior log, const LinearBooleanProblem& problem, SatSolver* solver,
    std::vector<bool>* solution);

// This is an original algorithm. It is a mix between the cardinality encoding
// and the Fu & Malik algorithm. It also works on general weighted instances.
SatSolver::Status SolveWithCardinalityEncodingAndCore(
    LogBehavior log, const LinearBooleanProblem& problem, SatSolver* solver,
    std::vector<bool>* solution);

// Model-based API, for now we just provide a basic algorithm that minimizes a
// given IntegerVariable by solving a sequence of decision problem by using
// SolveIntegerProblem().
//
// The feasible_solution_observer function will be called each time a new
// feasible solution is found.
SatSolver::Status MinimizeIntegerVariableWithLinearScanAndLazyEncoding(
    IntegerVariable objective_var,
    const std::function<void()>& feasible_solution_observer, Model* model);

// Use a low conflict limit and performs a binary search to try to restrict the
// domain of objective_var.
void RestrictObjectiveDomainWithBinarySearch(
    IntegerVariable objective_var,
    const std::function<void()>& feasible_solution_observer, Model* model);

// Same as MinimizeIntegerVariableWithLinearScanAndLazyEncoding() but use
// a core-based approach instead. Note that the given objective_var is just used
// for reporting the lower-bound/upper-bound and do not need to be linked with
// its linear representation.
//
// Unlike MinimizeIntegerVariableWithLinearScanAndLazyEncoding() this function
// just return the last solver status. In particular if it is INFEASIBLE but
// feasible_solution_observer() was called, it means we are at OPTIMAL.
SatSolver::Status MinimizeWithCoreAndLazyEncoding(
    IntegerVariable objective_var,
    const std::vector<IntegerVariable>& variables,
    const std::vector<IntegerValue>& coefficients,
    const std::function<void()>& feasible_solution_observer, Model* model);

// Generalization of the max-HS algorithm (HS stands for Hitting Set). This is
// similar to MinimizeWithCoreAndLazyEncoding() but it uses a hybrid approach
// with a MIP solver to handle the discovered infeasibility cores.
//
// See, Jessica Davies and Fahiem Bacchus, "Solving MAXSAT by Solving a
// Sequence of Simpler SAT Instances",
// http://www.cs.toronto.edu/~jdavies/daviesCP11.pdf"
//
// Note that an implementation of this approach won the 2016 max-SAT competition
// on the industrial category, see
// http://maxsat.ia.udl.cat/results/#wpms-industrial
//
// TODO(user): This function brings dependency to the SCIP MIP solver which is
// quite big, maybe we should find a way not to do that.
SatSolver::Status MinimizeWithHittingSetAndLazyEncoding(
    IntegerVariable objective_var, std::vector<IntegerVariable> variables,
    std::vector<IntegerValue> coefficients,
    const std::function<void()>& feasible_solution_observer, Model* model);

}  // namespace sat
}  // namespace operations_research

#endif  // OR_TOOLS_SAT_OPTIMIZATION_H_
