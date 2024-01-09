//
// Created by caochao on 7/12/19.
//

#include "../../include/tsp_solver/tsp_solver.h"
#include "utils/misc_utils.h"

namespace tsp_solver_ns
{
TSPSolver::TSPSolver(tsp_solver_ns::DataModel data) : data_(std::move(data))
{
  // Create Routing Index Manager
  if (data_.distance_matrix.empty() && data_.distance_matrices.empty())
  {
    exit(1);
  }
  int num_nodes = data_.distance_matrix.empty() ? data_.distance_matrices.front().size() : data_.distance_matrix.size();
  manager_ = std::make_unique<RoutingIndexManager>(num_nodes, data_.num_vehicles, data_.depots, data_.depots);

  // Create Routing Model.
  routing_ = std::make_unique<RoutingModel>(*manager_);
}

void TSPSolver::Solve()
{
  const int transit_callback_index =
      routing_->RegisterTransitCallback([this](int64 from_index, int64 to_index) -> int64 {
        // Convert from routing variable Index to distance matrix NodeIndex.
        auto from_node = manager_->IndexToNode(from_index).value();
        auto to_node = manager_->IndexToNode(to_index).value();
        return data_.distance_matrix[from_node][to_node];
      });

  // Define cost of each arc.
  routing_->SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  // Setting first solution heuristic.
  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  // Solve the problem.
  solution_ = routing_->SolveWithParameters(searchParameters);
}

void TSPSolver::SolveVRP(int64 max_route_length, bool time_window_constraint)
{
  std::vector<int> transit_callback_indices;
  for (int i = 0; i < data_.num_vehicles; i++)
  {
    const int transit_callback_index =
        routing_->RegisterTransitCallback([i, this](int64 from_index, int64 to_index) -> int64 {
          // Convert from routing variable Index to distance matrix NodeIndex.
          auto from_node = manager_->IndexToNode(from_index).value();
          auto to_node = manager_->IndexToNode(to_index).value();
          return data_.distance_matrices[i][from_node][to_node];
        });
    transit_callback_indices.push_back(transit_callback_index);
  }

  // Define cost of each arc.
  for (int i = 0; i < data_.num_vehicles; i++)
  {
    routing_->SetArcCostEvaluatorOfVehicle(transit_callback_indices[i], i);
  }

  routing_->AddDimensionWithVehicleTransits(transit_callback_indices, int64_t{ 0 }, max_route_length, true, "Distance");
  routing_->GetMutableDimension("Distance")->SetGlobalSpanCostCoefficient(100);

  if (time_window_constraint && !data_.time_windows.empty())
  {
    // Add time window constraints for each location except depot.
    const operations_research::RoutingDimension& time_dimension = routing_->GetDimensionOrDie("Distance");
    for (int i = data_.num_vehicles; i < data_.time_windows.size(); ++i)
    {
      int64_t index = manager_->NodeToIndex(operations_research::RoutingIndexManager::NodeIndex(i));
      time_dimension.CumulVar(index)->SetRange(data_.time_windows[i].first, data_.time_windows[i].second);
    }
    // Add time window constraints for each vehicle start node.
    for (int i = 0; i < data_.num_vehicles; ++i)
    {
      int64_t index = routing_->Start(i);
      time_dimension.CumulVar(index)->SetRange(data_.time_windows[i].first, data_.time_windows[i].second);
    }

    // Instantiate route start and end times to produce feasible times.
    for (int i = 0; i < data_.num_vehicles; ++i)
    {
      routing_->AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing_->Start(i)));
      routing_->AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing_->End(i)));
    }
  }

  // Allow to drop nodes
  int64_t penalty{ misc_utils_ns::INF_DISTANCE / 10 };
  int node_num = data_.distance_matrices.front().size();
  for (int i = 0; i < node_num; ++i)
  {
    routing_->AddDisjunction({ manager_->NodeToIndex(operations_research::RoutingIndexManager::NodeIndex(i)) },
                             penalty);
  }

  // Setting first solution heuristic.
  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
  searchParameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  // searchParameters.mutable_time_limit()->set_seconds(3);
  searchParameters.mutable_time_limit()->set_nanos(80000000);
  routing_->CloseModelWithParameters(searchParameters);

  // Solve the problem.
  // solution_ = routing_->SolveWithParameters(searchParameters);
  // std::cout << "Without initial solution cost: " << solution_->ObjectiveValue() << std::endl;
  // std::cout << "getting initial solution" << std::endl;

  std::vector<std::vector<int64_t>> initial_routes;
  initial_routes.reserve(data_.initial_routes.size());
  for (auto&& v : data_.initial_routes)
  {
    initial_routes.emplace_back(std::begin(v), std::end(v));
  }

  const operations_research::Assignment* initial_solution = routing_->ReadAssignmentFromRoutes(initial_routes, true);
  if (data_.initial_routes.empty() || initial_solution == nullptr)
  {
    // std::cout << "Solving without initial routes!" << std::endl;
    initial_objective_ = 0;
    solution_ = routing_->SolveWithParameters(searchParameters);
  }
  else
  {
    // std::cout << "Initial objective: " << initial_solution->ObjectiveValue() << std::endl;
    initial_objective_ = initial_solution->ObjectiveValue();
    solution_ = routing_->SolveFromAssignmentWithParameters(initial_solution, searchParameters);
  }
  // std::cout << "Solution objective: " << solution_->ObjectiveValue() << std::endl;
  solution_objective_ = solution_->ObjectiveValue();
}

void TSPSolver::PrintSolution()
{
  // Inspect solution.
  std::cout << "Objective: " << (solution_->ObjectiveValue()) << " meters" << std::endl;
  int64 index = routing_->Start(0);
  std::cout << "Route:";
  int64 distance{ 0 };
  std::stringstream route;
  while (routing_->IsEnd(index) == false)
  {
    route << manager_->IndexToNode(index).value() << " -> ";
    int64 previous_index = index;
    index = solution_->Value(routing_->NextVar(index));
    distance += const_cast<RoutingModel&>(*routing_).GetArcCostForVehicle(previous_index, index, 0LL);
  }
  std::cout << route.str() << manager_->IndexToNode(index).value() << std::endl;
  std::cout << "Route distance: " << distance << " meters" << std::endl;
  std::cout << "Problem solved in " << routing_->solver()->wall_time() << "ms" << std::endl;
}

void TSPSolver::PrintSolution(DataModel& data, RoutingIndexManager& manager, RoutingModel& routing,
                              const Assignment& solution)
{
  // Display dropped nodes.
  std::ostringstream dropped_nodes;
  for (int64_t node = 0; node < routing.Size(); ++node)
  {
    if (routing.IsStart(node) || routing.IsEnd(node))
      continue;
    if (solution.Value(routing.NextVar(node)) == node)
    {
      dropped_nodes << " " << manager.IndexToNode(node).value();
    }
  }
  std::cout << "Dropped nodes:" << dropped_nodes.str() << std::endl;

  int64_t total_distance{ 0 };
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id)
  {
    int64_t index = routing.Start(vehicle_id);
    std::cout << "V" << vehicle_id << ": ";
    int64_t route_distance{ 0 };
    std::stringstream route;
    while (routing.IsEnd(index) == false)
    {
      int64_t node_index = manager.IndexToNode(index).value();
      route << node_index << " -> ";
      int64_t previous_index = index;
      index = solution.Value(routing.NextVar(index));
      route_distance += routing.GetArcCostForVehicle(previous_index, index, int64_t{ vehicle_id });
    }
    std::cout << route.str() << manager.IndexToNode(index).value();
    std::cout << " (" << route_distance << "m)" << std::endl;
    total_distance += route_distance;
  }
  std::cout << "Total distance of all routes: " << total_distance << "m" << std::endl;
  std::cout << "Problem solved in " << routing.solver()->wall_time() << "ms" << std::endl;
}

void TSPSolver::PrintVRPSolution()
{
  // Display dropped nodes.
  std::ostringstream dropped_nodes;
  for (int64_t node = 0; node < routing_->Size(); ++node)
  {
    if (routing_->IsStart(node) || routing_->IsEnd(node))
      continue;
    if (solution_->Value(routing_->NextVar(node)) == node)
    {
      dropped_nodes << " " << manager_->IndexToNode(node).value();
    }
  }
  std::cout << "Dropped nodes:" << dropped_nodes.str() << std::endl;

  int64_t total_distance{ 0 };
  int64_t total_load{ 0 };
  for (int vehicle_id = 0; vehicle_id < data_.num_vehicles; ++vehicle_id)
  {
    int64_t index = routing_->Start(vehicle_id);
    std::cout << "V" << vehicle_id << ": ";
    int64_t route_distance{ 0 };
    std::stringstream route;
    while (routing_->IsEnd(index) == false)
    {
      int64_t node_index = manager_->IndexToNode(index).value();
      route << node_index << " -> ";
      int64_t previous_index = index;
      index = solution_->Value(routing_->NextVar(index));
      route_distance += routing_->GetArcCostForVehicle(previous_index, index, int64_t{ vehicle_id });
    }
    std::cout << route.str() << manager_->IndexToNode(index).value();
    std::cout << " (" << route_distance << "m)" << std::endl;
    total_distance += route_distance;
  }
  std::cout << "Total distance of all routes: " << total_distance << "m" << std::endl;
  std::cout << "Problem solved in " << routing_->solver()->wall_time() << "ms" << std::endl;
}

int TSPSolver::getComputationTime()
{
  return routing_->solver()->wall_time();
}

void TSPSolver::getSolutionNodeIndex(std::vector<int>& node_index, bool has_dummy)
{
  node_index.clear();
  int index = routing_->Start(0);
  int end_index = index;
  while (routing_->IsEnd(index) == false)
  {
    node_index.push_back(manager_->IndexToNode(index).value());
    index = solution_->Value(routing_->NextVar(index));
  }
  // push back the end node index
  //       node_index.push_back(end_index);
  if (has_dummy)
  {
    int dummy_node_index = data_.distance_matrix.size() - 1;
    if (node_index[1] == dummy_node_index)
    {
      // delete dummy node
      node_index.erase(node_index.begin() + 1);
      // push the start node to the end
      node_index.push_back(node_index[0]);
      // remove the start node at the begining
      node_index.erase(node_index.begin());
      // reverse the whole array
      std::reverse(node_index.begin(), node_index.end());
    }
    else  // the last node is dummy node
    {
      node_index.pop_back();
    }
  }
}

void TSPSolver::GetVRPSolutionNodeIndex(std::vector<std::vector<int>>& ordered_node_indices)
{
  ordered_node_indices.clear();

  for (int vehicle_id = 0; vehicle_id < data_.num_vehicles; vehicle_id++)
  {
    std::vector<int> vehicle_ordered_node_indices;
    int index = routing_->Start(vehicle_id);
    while (routing_->IsEnd(index) == false)
    {
      int node_index = manager_->IndexToNode(index).value();
      vehicle_ordered_node_indices.push_back(node_index);
      index = solution_->Value(routing_->NextVar(index));
    }
    // Push back the first node to make it a loop
    if (!vehicle_ordered_node_indices.empty())
    {
      vehicle_ordered_node_indices.push_back(vehicle_ordered_node_indices.front());

      // // Reorder the list to make the shorter-distance-node-pair at the front
      // int from_node_index = vehicle_ordered_node_indices[0];
      // int to_node_index = vehicle_ordered_node_indices[1];
      // int distance_at_front = data_.distance_matrices[vehicle_id][from_node_index][to_node_index];

      // from_node_index = vehicle_ordered_node_indices[vehicle_ordered_node_indices.size() - 1];
      // to_node_index = vehicle_ordered_node_indices[vehicle_ordered_node_indices.size() - 2];
      // int distance_at_end = data_.distance_matrices[vehicle_id][from_node_index][to_node_index];
      // if (distance_at_end < distance_at_front)
      // {
      //   std::reverse(vehicle_ordered_node_indices.begin(), vehicle_ordered_node_indices.end());
      // }
    }

    ordered_node_indices.push_back(vehicle_ordered_node_indices);
  }
}

int64_t TSPSolver::GetSolutionObjective()
{
  return solution_objective_;
}

int64_t TSPSolver::GetInitialObjective()
{
  return initial_objective_;
}

int TSPSolver::GetDroppedNodesNumber()
{
  int dropped_nodes_num = 0;
  for (int i = 0; i < routing_->Size(); i++)
  {
    if (routing_->IsStart(i) || routing_->IsEnd(i))
    {
      continue;
    }
    if (solution_->Value(routing_->NextVar(i)) == i)
    {
      dropped_nodes_num++;
    }
  }
  return dropped_nodes_num;
}

}  // namespace tsp_solver_ns
