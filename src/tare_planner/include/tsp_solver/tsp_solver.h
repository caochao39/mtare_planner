//
// Created by caochao on 7/12/19.
//

#ifndef VISUAL_COVERAGE_PLANNER_TSP_SOLVER_H
#define VISUAL_COVERAGE_PLANNER_TSP_SOLVER_H
#include <cmath>
#include <vector>
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

using namespace operations_research;

namespace tsp_solver_ns
{
struct DataModel;
class TSPSolver;
}  // namespace tsp_solver_ns

struct tsp_solver_ns::DataModel
{
  std::vector<std::vector<int>> initial_routes;
  std::vector<std::vector<int>> distance_matrix;
  std::vector<std::vector<std::vector<int>>> distance_matrices;
  int num_vehicles = 1;
  RoutingIndexManager::NodeIndex depot{ 0 };
  std::vector<RoutingIndexManager::NodeIndex> depots;
  std::vector<std::pair<int, int>> time_windows;
};

class tsp_solver_ns::TSPSolver
{
private:
  DataModel data_;
  std::unique_ptr<RoutingIndexManager> manager_;
  std::unique_ptr<RoutingModel> routing_;
  const Assignment* solution_;
  int64_t initial_objective_;
  int64_t solution_objective_;

public:
  TSPSolver(DataModel data);
  ~TSPSolver() = default;
  void Solve();
  void SolveVRP(int64 max_route_length = INT_MAX, bool time_window_constraint = false);
  void SolveVRPWithTimeWindowConstraint();
  void PrintSolution();
  void PrintSolution(DataModel& data, RoutingIndexManager& manager, RoutingModel& routing, const Assignment& solution);
  void PrintVRPSolution();
  int getComputationTime();
  void getSolutionNodeIndex(std::vector<int>& node_index, bool has_dummy);
  void GetVRPSolutionNodeIndex(std::vector<std::vector<int>>& ordered_node_indices);
  int64_t GetSolutionObjective();
  int64_t GetInitialObjective();
  int GetDroppedNodesNumber();
};

#endif  // VISUAL_COVERAGE_PLANNER_TSP_SOLVER_H
