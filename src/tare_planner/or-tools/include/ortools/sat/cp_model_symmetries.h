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

#ifndef OR_TOOLS_SAT_CP_MODEL_SYMMETRIES_H_
#define OR_TOOLS_SAT_CP_MODEL_SYMMETRIES_H_

#include <limits>
#include <memory>
#include <vector>

#include "ortools/algorithms/sparse_permutation.h"
#include "ortools/sat/cp_model.pb.h"

namespace operations_research {
namespace sat {

// Returns a list of generators of the symmetry group of the given problem. Each
// generator is a permutation of the integer range [0, n) where n is the number
// of variables of the problem. They are permutations of the (index
// representation of the) problem variables.
void FindCpModelSymmetries(
    const CpModelProto& problem,
    std::vector<std::unique_ptr<SparsePermutation>>* generators,
    double time_limit_seconds = std::numeric_limits<double>::infinity());

}  // namespace sat
}  // namespace operations_research

#endif  // OR_TOOLS_SAT_CP_MODEL_SYMMETRIES_H_
