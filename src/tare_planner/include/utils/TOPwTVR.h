/**
 * @file TOPwTVR.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that solves a Team Orienteering Problem with Time-Varying Reward problem
 * @version 0.1
 * @date 2023-04-07
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <functional>  // for std::hash
#include <utils/TOPwTVR_utils.h>
#include <utils/PBS_utils.h>
#include <utils/CBS_utils.h>

enum HeuristicType
{
  FLAT,
  SIMPLE,
  KNAPSACK
};

struct Node
{
  int id_;        // indicate a node on the graph
  int label_id_;  // indicate a state in the search tree
  double g_;      // reward to come
  double f_;      // estimated reward f = g + heuristic
  int t_;
  IndexSet visited_nodes_;
  int parent_;
  int parent_iteration_id_;  // For debug

  Node();
  Node(int id, int label_id, double g, double f, int t, int node_num);
  void AddVisitedNodeID(int node_id)
  {
    // visited_nodes_ |= 1 << node_id;
    visited_nodes_.Insert(node_id);
  }
  void SetF(double f)
  {
    f_ = f;
  }
  void SetG(double g)
  {
    g_ = g;
  }
  void SetT(int t)
  {
    t_ = t;
  }
};

struct JointStateNode
{
  int vehicle_num_;
  std::vector<Node> nodes_;
  double f_;
  double g_;
  double t_;
  int parent_iteration_id_;
  int label_id_;
  int parent_;
  int move_vehicle_id_;

  JointStateNode(int vehicle_num);
  JointStateNode(const std::vector<Node>& nodes);

  std::string GetLabel() const;
  void UpdateF();
  void UpdateG();
  void UpdateT();
  void UpdateFGT();
  double GetF()
  {
    return f_;
  }
  double GetG()
  {
    return g_;
  }
  void SetG(double g)
  {
    g_ = g;
  }
  int GetT()
  {
    return t_;
  }
  void DiscountF(double alpha)
  {
    f_ *= alpha;
  }
  // Modify a specific node
  void SetNode(int node_id, const Node& node);
  void SetNodeF(int node_id, double f);
  void SetNodeG(int node_id, double g);
  void SetNodeT(int node_id, int t);
  void AddVisitedNodeIDToNode(int node_id, int visited_node_id);

  bool operator==(const JointStateNode& other) const
  {
    for (int i = 0; i < vehicle_num_; i++)
    {
      if (nodes_[i].id_ != other.nodes_[i].id_ || nodes_[i].t_ != other.nodes_[i].t_ ||
          nodes_[i].visited_nodes_ != other.nodes_[i].visited_nodes_)
      {
        return false;
      }
    }
    return true;
  }
};

struct CompareNode
{
  bool operator()(const Node& a, const Node& b) const
  {
    // Consider f first
    if (a.f_ == b.f_)
    {
      if (a.t_ == b.t_)
      {
        return a.parent_iteration_id_ > b.parent_iteration_id_;
      }
      return a.t_ > b.t_;  // In case of a tie, compare by the second key 't' (smaller t value first)
    }
    return a.f_ < b.f_;  // Sort by the first key 'f' (bigger f value first)

    // // Consider t first
    // if (a.t_ == b.t_)
    // {
    //   if (a.f_ == b.f_)
    //   {
    //     return a.parent_iteration_id_ > b.parent_iteration_id_;
    //   }
    //   return a.f_ < b.f_;
    // }
    // return a.t_ > b.t_;

    // // Consider t first
    // if (a.t_ == b.t_)
    // {
    //   if (a.id_ == b.id_)
    //   {
    //     // return a.parent_ > b.parent_;
    //     return a.parent_iteration_id_ > b.parent_iteration_id_;
    //   }
    //   return a.id_ > b.id_;
    // }
    // return a.t_ > b.t_;
  }
};

struct CompareJointStateNode
{
  bool operator()(const JointStateNode& a, const JointStateNode& b) const
  {
    // Consider f first
    if (a.f_ == b.f_)
    {
      if (a.t_ == b.t_)
      {
        return a.parent_iteration_id_ > b.parent_iteration_id_;
      }
      return a.t_ > b.t_;  // In case of a tie, compare by the second key 't' (smaller t value first)
    }
    return a.f_ < b.f_;  // Sort by the first key 'f' (bigger f value first)
  }
};

class TeamOrienteeringProblemSolver
{
  void Ind2Sub(int ind, int& node_id, int& t);
  int Sub2Ind(int node_id, int t);
  int GetNodeIndex(Node node);

  bool IsSubset(int set1, int set2);
  bool IsPruned(const Node& node, const Node& best_so_far_node);
  bool IsPruned(const JointStateNode& node, const JointStateNode& best_so_far_node,
                std::unordered_map<std::string, JointStateNode>& frontier, std::unordered_set<std::string>& closed);
  void AddToFrontier(const Node& node);
  void AddToClosed(const Node& node);

  void AddToFrontier(const JointStateNode& node, std::unordered_map<std::string, JointStateNode>& frontier);
  void AddToClosed(const JointStateNode& node, std::unordered_set<std::string>& closed);

  double GetHeuristicValue(Node node);

  void ProcessJointStateNodeNeighbors(
      const JointStateNode& curr_joint_state_node, const JointStateNode& best_so_far_joint_state_node,
      std::unordered_map<std::string, JointStateNode>& frontier, std::unordered_set<std::string>& closed,
      std::priority_queue<JointStateNode, std::vector<JointStateNode>, CompareJointStateNode>& pq,
      std::vector<JointStateNode>& all_nodes, int time_budget, int K, bool debug = false);

  int graph_node_num_;
  IndexSet node_set_;  // the set of nodes that can be visited by this vehicle
  int time_interval_num_;
  int vehicle_num_;
  int frontier_queue_size_;
  bool use_closed_set_;
  bool accumulate_reward_at_same_node_;
  int node_expanded_count_;
  int start_node_s_id_;
  int time_budget_;
  int runtime_limit_;
  std::vector<int> start_graph_node_ids_;
  std::vector<int> goal_graph_node_ids_;
  HeuristicType heuristic_type_;
  std::vector<int> parent_;
  std::vector<std::vector<int>> distance_matrix_;
  std::vector<std::vector<double>> profits_;
  std::vector<std::vector<double>> remaining_best_profits_;
  std::vector<double> remaining_best_profits_all_nodes_;
  std::vector<std::vector<Node>> frontier_;
  std::vector<bool> closed_;
  std::vector<Node> all_nodes_;

public:
  TeamOrienteeringProblemSolver(int node_num, int vehicle_num, int time_interval_num,
                                const std::vector<std::vector<int>>& distance_matrix,
                                const std::vector<std::vector<double>>& profits);
  ~TeamOrienteeringProblemSolver() = default;
  void SetTimeBudget(int time_budget)
  {
    time_budget_ = time_budget;
  }
  void SetRuntimeLimit(int runtime_limit)
  {
    runtime_limit_ = runtime_limit;
  }
  void SetStartGraphNodeIDs(const std::vector<int>& start_graph_node_ids)
  {
    start_graph_node_ids_ = start_graph_node_ids;
  }
  void SetGoalGraphNodeIDs(const std::vector<int>& goal_graph_node_ids)
  {
    goal_graph_node_ids_ = goal_graph_node_ids;
  }
  void SetFrontierQueueSize(int frontier_queue_size)
  {
    frontier_queue_size_ = frontier_queue_size;
  }
  void SetUseClosedSet(bool use_closed_set)
  {
    use_closed_set_ = use_closed_set;
  }
  void SetAccumulateRewardAtSameNode(bool accumulate_reward_at_same_node)
  {
    accumulate_reward_at_same_node_ = accumulate_reward_at_same_node;
  }
  void SetHeuristicType(HeuristicType heuristic_type)
  {
    heuristic_type_ = heuristic_type;
  }
  void IncludeAllNodes()
  {
    node_set_.IncludeAllElements();
  }
  void ExcludeNodes(const std::vector<int>& node_ids)
  {
    for (const auto& node_id : node_ids)
    {
      node_set_.Remove(node_id);
    }
  }
  void ExcludeNode(int node_id)
  {
    node_set_.Remove(node_id);
  }

  void IncludeNode(int node_id)
  {
    node_set_.Insert(node_id);
  }

  void PrintIncludedNodes()
  {
    node_set_.Print();
  }

  bool CanBeVisited(int node_id);

  double SolveSingleVehicle(int start_graph_node_id, int goal_graph_node_id, std::vector<int>& path);
  double SolveMultiVehicle(int start_node_id, int end_goal_id, std::vector<std::vector<int>>& paths,
                           int K);  // runtime_limit in s
  double SolveMultiVehicleGreedy(const std::vector<int>& start_node_ids, std::vector<int>& end_goal_ids,
                                 std::vector<std::vector<int>>& paths);
  void GeneratePBSRootNode(PBS_ns::PBS& pbs);
  void GeneratePBSChildNode(PBS_ns::PBS& pbs, int child_id, PBS_ns::PBSNode* parent, int low, int high);
  void PushPBSNodes(PBS_ns::PBS& pbs, PBS_ns::PBSNode* n1, PBS_ns::PBSNode* n2);
  bool FindPathForSingleAgent(PBS_ns::PBS& pbs, PBS_ns::PBSNode& node, const std::set<int>& higher_agents, int a,
                              std::vector<int>& new_path);
  double SolveMultiVehiclePBS(std::vector<std::vector<int>>& paths);

  void ComputeCBSNodeHeuristic(CBS_ns::CBSNode& node);
  void GenerateCBSRootNode(CBS_ns::CBS& cbs);
  void AddCBSConstraints(const CBS_ns::CBSNode& node, CBS_ns::CBSNode* child1, CBS_ns::CBSNode* child2);
  void GetInvalidAgentsAndExcludingNodes(const CBS_ns::CBSNode& node, std::vector<int>& invalid_agents,
                                         std::vector<std::vector<int>>& excluding_nodes);
  void GenerateCBSChildNode(CBS_ns::CBSNode* node, CBS_ns::CBSNode* parent);
  void PushCBSNodes(CBS_ns::CBS& cbs, CBS_ns::CBSNode* n1, CBS_ns::CBSNode* n2);
  void GetConflictingNodes(CBS_ns::CBSNode* node, int a1, int a2, std::vector<int>& conflicting_nodes);
  void FindConflicts(CBS_ns::CBSNode& node);
  bool FindPathForSingleAgent(CBS_ns::CBSNode& node, int vehicle_id,
                              const std::vector<std::vector<int>>& excluding_nodes);
  double SolveMultiVehicleCBS(std::vector<std::vector<int>>& paths);

  int GetNodeExpandedCount()
  {
    return node_expanded_count_;
  }
  // Utility functions

  // Printing for debugging
  void PrintNode(const Node& node);
  void PrintNode(const JointStateNode& node);
  void PrintNode(const PBS_ns::PBSNode& node);
  void PrintNode(const CBS_ns::CBSNode& node);
  std::string GetVisitedNodesString(IndexSet visited_nodes);
};

namespace std
{
template <>
struct hash<JointStateNode>
{
  std::size_t operator()(const JointStateNode& p) const
  {
    return std::hash<std::string>()(p.GetLabel());
  }
};
}  // namespace std