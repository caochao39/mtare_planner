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

#ifndef OR_TOOLS_CONSTRAINT_SOLVER_ROUTING_FLAGS_H_
#define OR_TOOLS_CONSTRAINT_SOLVER_ROUTING_FLAGS_H_

#include <vector>

#include "ortools/base/commandlineflags.h"
#include "ortools/constraint_solver/routing_parameters.pb.h"

// --- Routing search flags ---

// Neighborhood activation/deactivation
DECLARE_bool(routing_no_lns);
DECLARE_bool(routing_no_fullpathlns);
DECLARE_bool(routing_no_relocate);
DECLARE_bool(routing_no_relocate_neighbors);
DECLARE_bool(routing_no_relocate_subtrip);
DECLARE_bool(routing_no_exchange);
DECLARE_bool(routing_no_exchange_subtrip);
DECLARE_bool(routing_no_cross);
DECLARE_bool(routing_no_2opt);
DECLARE_bool(routing_no_oropt);
DECLARE_bool(routing_no_make_active);
DECLARE_bool(routing_no_lkh);
DECLARE_bool(routing_no_relocate_expensive_chain);
DECLARE_bool(routing_no_tsp);
DECLARE_bool(routing_no_tsplns);
DECLARE_bool(routing_use_chain_make_inactive);
DECLARE_bool(routing_use_extended_swap_active);

// Meta-heuristics
DECLARE_bool(routing_guided_local_search);
DECLARE_double(routing_guided_local_search_lambda_coefficient);
DECLARE_bool(routing_simulated_annealing);
DECLARE_bool(routing_tabu_search);
DECLARE_bool(routing_generic_tabu_search);

// Search limits
DECLARE_int64(routing_solution_limit);
DECLARE_int64(routing_time_limit);
DECLARE_int64(routing_lns_time_limit);

// Search control
DECLARE_string(routing_first_solution);
DECLARE_bool(routing_use_filtered_first_solutions);
DECLARE_double(savings_neighbors_ratio);
DECLARE_bool(savings_add_reverse_arcs);
DECLARE_double(savings_arc_coefficient);
DECLARE_double(cheapest_insertion_farthest_seeds_ratio);
DECLARE_double(cheapest_insertion_neighbors_ratio);
DECLARE_bool(routing_dfs);
DECLARE_double(routing_optimization_step);
DECLARE_int32(routing_number_of_solutions_to_collect);
DECLARE_int32(routing_relocate_expensive_chain_num_arcs_to_consider);

// Propagation control
DECLARE_bool(routing_use_light_propagation);

// Cache settings.
DECLARE_bool(routing_cache_callbacks);
DECLARE_int64(routing_max_cache_size);

// Misc
DECLARE_bool(routing_trace);
DECLARE_bool(routing_profile);

// --- Routing model flags ---
DECLARE_bool(routing_use_homogeneous_costs);
DECLARE_bool(routing_gzip_compress_trail);

namespace operations_research {

// Builds routing search parameters from flags.
RoutingModelParameters BuildModelParametersFromFlags();

// Builds routing search parameters from flags.
// TODO(user): Make this return a StatusOr, verifying that the flags
// describe a valid set of routing search parameters.
RoutingSearchParameters BuildSearchParametersFromFlags();

}  // namespace operations_research

#endif  // OR_TOOLS_CONSTRAINT_SOLVER_ROUTING_FLAGS_H_
