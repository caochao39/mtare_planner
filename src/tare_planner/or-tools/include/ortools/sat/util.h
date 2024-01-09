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

#ifndef OR_TOOLS_SAT_UTIL_H_
#define OR_TOOLS_SAT_UTIL_H_

#include "ortools/base/random.h"
#include "ortools/sat/model.h"
#include "ortools/sat/sat_base.h"
#include "ortools/sat/sat_parameters.pb.h"
#include "ortools/util/random_engine.h"

#if !defined(__PORTABLE_PLATFORM__)
#include "google/protobuf/descriptor.h"
#endif  // __PORTABLE_PLATFORM__

namespace operations_research {
namespace sat {

// The model "singleton" random engine used in the solver.
struct ModelRandomGenerator : public random_engine_t {
  // We seed the strategy at creation only. This should be enough for our use
  // case since the SatParameters is set first before the solver is created. We
  // also never really need to change the seed afterwards, it is just used to
  // diversify solves with identical parameters on different Model objects.
  explicit ModelRandomGenerator(Model* model) : random_engine_t() {
    seed(model->GetOrCreate<SatParameters>()->random_seed());
  }
};

// Randomizes the decision heuristic of the given SatParameters.
template <typename URBG>
void RandomizeDecisionHeuristic(URBG* random, SatParameters* parameters);

// Context: this function is not really generic, but required to be unit-tested.
// It is used in a clause minimization algorithm when we try to detect if any of
// the clause literals can be propagated by a subset of the other literal being
// false. For that, we want to enqueue in the solver all the subset of size n-1.
//
// This moves one of the unprocessed literal from literals to the last position.
// The function tries to do that while preserving the longest possible prefix of
// literals "amortized" through the calls assuming that we want to move each
// literal to the last position once.
//
// For a vector of size n, if we want to call this n times so that each literal
// is last at least once, the sum of the size of the changed suffixes will be
// O(n log n). If we were to use a simpler algorithm (like moving the last
// unprocessed literal to the last position), this sum would be O(n^2).
//
// Returns the size of the common prefix of literals before and after the move,
// or -1 if all the literals are already processed. The argument
// relevant_prefix_size is used as a hint when keeping more that this prefix
// size do not matter. The returned value will always be lower or equal to
// relevant_prefix_size.
int MoveOneUnprocessedLiteralLast(const std::set<LiteralIndex>& processed,
                                  int relevant_prefix_size,
                                  std::vector<Literal>* literals);

// ============================================================================
// Implementation.
// ============================================================================

template <typename URBG>
inline void RandomizeDecisionHeuristic(URBG* random,
                                       SatParameters* parameters) {
#if !defined(__PORTABLE_PLATFORM__)
  // Random preferred variable order.
  const google::protobuf::EnumDescriptor* order_d =
      SatParameters::VariableOrder_descriptor();
  parameters->set_preferred_variable_order(
      static_cast<SatParameters::VariableOrder>(
          order_d->value(absl::Uniform(*random, 0, order_d->value_count()))
              ->number()));

  // Random polarity initial value.
  const google::protobuf::EnumDescriptor* polarity_d =
      SatParameters::Polarity_descriptor();
  parameters->set_initial_polarity(static_cast<SatParameters::Polarity>(
      polarity_d->value(absl::Uniform(*random, 0, polarity_d->value_count()))
          ->number()));
#endif  // __PORTABLE_PLATFORM__
  // Other random parameters.
  parameters->set_use_phase_saving(absl::Bernoulli(*random, 0.5));
  parameters->set_random_polarity_ratio(absl::Bernoulli(*random, 0.5) ? 0.01
                                                                      : 0.0);
  parameters->set_random_branches_ratio(absl::Bernoulli(*random, 0.5) ? 0.01
                                                                      : 0.0);
}

// Manages incremental averages.
class IncrementalAverage {
 public:
  // Initializes the average with 'initial_average' and number of records to 0.
  explicit IncrementalAverage(double initial_average)
      : average_(initial_average) {}
  IncrementalAverage() {}

  // Sets the number of records to 0 and average to 'reset_value'.
  void Reset(double reset_value);

  double CurrentAverage() const { return average_; }
  int64 NumRecords() const { return num_records_; }

  void AddData(double new_record);

 private:
  double average_ = 0.0;
  int64 num_records_ = 0;
};

}  // namespace sat
}  // namespace operations_research

#endif  // OR_TOOLS_SAT_UTIL_H_
