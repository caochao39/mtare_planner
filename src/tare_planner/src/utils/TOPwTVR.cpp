/**
 * @file TOPwTVR_test.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Solve a Team Orienteering Problem with Time-Varying Reward problem
 * @version 0.1
 * @date 2023-04-08
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <vector>
#include <cmath>
#include <queue>
#include <stack>
#include <unordered_map>
#include <iostream>
#include <functional>
#include <climits>
#include <algorithm>
#include <utils/TOPwTVR.h>
#include <utils/misc_utils.h>

Node::Node()
  : id_(-1), label_id_(-1), g_(0.0), f_(0.0), t_(0), visited_nodes_(300), parent_(-1), parent_iteration_id_(-1)
{
}

Node::Node(int id, int label_id, double g, double f, int t, int node_num)
  : id_(id), label_id_(label_id), g_(g), f_(f), t_(t), visited_nodes_(node_num), parent_(-1), parent_iteration_id_(-1)
{
}

JointStateNode::JointStateNode(int vehicle_num)
  : vehicle_num_(vehicle_num), parent_iteration_id_(-1), label_id_(-1), parent_(-1), move_vehicle_id_(-1)
{
  for (int i = 0; i < vehicle_num; i++)
  {
    Node node;
    nodes_.push_back(node);
  }
}

JointStateNode::JointStateNode(const std::vector<Node>& nodes)
  : vehicle_num_(nodes.size())
  , nodes_(nodes)
  , parent_iteration_id_(-1)
  , label_id_(-1)
  , parent_(-1)
  , move_vehicle_id_(-1)
{
  UpdateFGT();
}

std::string JointStateNode::GetLabel() const
{
  std::string label;
  for (const auto& node : nodes_)
  {
    label += (std::to_string(node.id_) + std::to_string(node.t_));
  }
  return label;
}

void JointStateNode::UpdateF()
{
  f_ = 0.0;
  for (const auto& node : nodes_)
  {
    f_ += node.f_;
  }
}
void JointStateNode::UpdateG()
{
  g_ = 0.0;
  for (const auto& node : nodes_)
  {
    g_ += node.g_;
  }
}
void JointStateNode::UpdateT()
{
  t_ = INT_MAX;
  for (const auto& node : nodes_)
  {
    t_ = node.t_ < t_ ? node.t_ : t_;
  }
}

void JointStateNode::UpdateFGT()
{
  f_ = 0.0;
  g_ = 0.0;
  t_ = INT_MAX;
  for (const auto& node : nodes_)
  {
    f_ += node.f_;
    g_ += node.g_;
    t_ = node.t_ < t_ ? node.t_ : t_;
  }
}

void JointStateNode::SetNode(int node_id, const Node& node)
{
  if (node_id >= 0 && node_id < vehicle_num_)
  {
    nodes_[node_id] = node;
    UpdateFGT();
  }
  else
  {
    std::cout << "Node id " << node_id << " out of bound" << std::endl;
    exit(1);
  }
}

void JointStateNode::SetNodeF(int node_id, double f)
{
  if (node_id >= 0 && node_id < vehicle_num_)
  {
    nodes_[node_id].SetF(f);
    UpdateF();
  }
  else
  {
    std::cout << "Node id " << node_id << " out of bound" << std::endl;
    exit(1);
  }
}

void JointStateNode::SetNodeG(int node_id, double g)
{
  if (node_id >= 0 && node_id < vehicle_num_)
  {
    nodes_[node_id].SetG(g);
    UpdateG();
  }
  else
  {
    std::cout << "Node id " << node_id << " out of bound" << std::endl;
    exit(1);
  }
}

void JointStateNode::SetNodeT(int node_id, int t)
{
  if (node_id >= 0 && node_id < vehicle_num_)
  {
    nodes_[node_id].SetT(t);
    UpdateT();
  }
  else
  {
    std::cout << "Node id " << node_id << " out of bound" << std::endl;
    exit(1);
  }
}

void JointStateNode::AddVisitedNodeIDToNode(int node_id, int visited_node_id)
{
  if (node_id >= 0 && node_id < vehicle_num_)
  {
    nodes_[node_id].AddVisitedNodeID(visited_node_id);
  }
  else
  {
    std::cout << "Node id " << node_id << " out of bound" << std::endl;
    exit(1);
  }
}

TeamOrienteeringProblemSolver::TeamOrienteeringProblemSolver(int node_num, int vehicle_num, int time_interval_num,
                                                             const std::vector<std::vector<int>>& distance_matrix,
                                                             const std::vector<std::vector<double>>& profits)
  : graph_node_num_(node_num)
  , vehicle_num_(vehicle_num)
  , time_interval_num_(time_interval_num)
  , distance_matrix_(distance_matrix)
  , profits_(profits)
  , frontier_queue_size_(-1)  // Infinite
  , use_closed_set_(false)
  , heuristic_type_(SIMPLE)
  , node_expanded_count_(0)
  , start_node_s_id_(0)
  , accumulate_reward_at_same_node_(true)
  , node_set_(node_num)
{
  // TODO: Check if the distance_matrix and profit dimension agrees with the node number and time interval number
  // if (node_num != distance_matrix.size() || node_num != profits.size())
  // {
  // }

  int n_num = profits_.size();
  int t_num = profits_[0].size();
  remaining_best_profits_.resize(n_num, std::vector<double>(t_num, 0.0));
  remaining_best_profits_all_nodes_.resize(t_num, 0.0);

  for (int t = t_num - 1; t >= 0; t--)
  {
    for (int node_id = 0; node_id < n_num; node_id++)
    {
      if (t == t_num - 1)
      {
        remaining_best_profits_[node_id][t] = profits_[node_id][t];
      }
      else
      {
        remaining_best_profits_[node_id][t] = std::max(profits_[node_id][t], remaining_best_profits_[node_id][t + 1]);
      }
      remaining_best_profits_all_nodes_[t] =
          std::max(remaining_best_profits_all_nodes_[t], remaining_best_profits_[node_id][t]);
    }
  }
  node_set_.IncludeAllElements();
}

void TeamOrienteeringProblemSolver::Ind2Sub(int ind, int& node_id, int& t)
{
  node_id = ind / time_interval_num_;
  t = ind % time_interval_num_;
}

int TeamOrienteeringProblemSolver::Sub2Ind(int node_id, int t)
{
  return node_id * time_interval_num_ + t;
}

int TeamOrienteeringProblemSolver::GetNodeIndex(Node node)
{
  return Sub2Ind(node.id_, node.t_);
}

bool TeamOrienteeringProblemSolver::IsSubset(int set1, int set2)
{
  // Check if set1 is a subset of set2
  return (set1 & set2) == set1;
}

bool TeamOrienteeringProblemSolver::IsPruned(const Node& node, const Node& best_so_far_node)
{
  bool debug = false;
  // if (node.id_ == 6 && node.t_ == 53)
  // {
  //   debug = true;
  // }

  if (best_so_far_node.id_ != -1 && node.f_ < best_so_far_node.g_)
  {
    if (debug)
    {
      std::cout << "dbg1111" << std::endl;
    }
    return true;
  }

  int index = GetNodeIndex(node);

  // k = 1, only keeping the best one
  if (use_closed_set_ && closed_[index])
  {
    if (debug)
    {
      std::cout << "dbg2222" << std::endl;
    }
    return true;
  }

  for (int i = 0; i < frontier_[index].size(); i++)
  {
    Node prev_node = frontier_[index][i];

    if (frontier_queue_size_ == -1)
    {
      // if (IsSubset(prev_node.visited_nodes_, node.visited_nodes_) && prev_node.g_ > node.g_)
      if (prev_node.visited_nodes_.IsSubset(node.visited_nodes_) && prev_node.g_ > node.g_)
      {
        return true;
      }
    }
    else if (frontier_queue_size_ == 1)
    {
      // k = 1, only keeping the best one with the largest g
      if (prev_node.g_ > node.g_)
      {
        if (debug)
        {
          std::cout << "dbg3333" << std::endl;
        }
        return true;
      }
    }
    else
    {
      // TODO
    }
  }
  return false;
}

bool TeamOrienteeringProblemSolver::IsPruned(const JointStateNode& node, const JointStateNode& best_so_far_node,
                                             std::unordered_map<std::string, JointStateNode>& frontier,
                                             std::unordered_set<std::string>& closed)
{
  if (best_so_far_node.parent_iteration_id_ != -1 && node.f_ < best_so_far_node.g_)
  {
    return true;
  }

  std::string node_label = node.GetLabel();

  if (use_closed_set_ && closed.find(node_label) != closed.end())
  {
    return true;
  }

  auto it = frontier.find(node_label);
  if (it == frontier.end())
  {
    return false;
  }

  if (it->second.g_ > node.g_)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void TeamOrienteeringProblemSolver::AddToFrontier(const Node& node)
{
  int index = GetNodeIndex(node);
  if (frontier_queue_size_ == -1)
  {
    frontier_[index].push_back(node);
  }
  else if (frontier_queue_size_ == 1)
  {
    // k = 1, only keeping the best one
    if (frontier_[index].empty())
    {
      frontier_[index].push_back(node);
    }
    else
    {
      if (node.g_ > frontier_[index][0].g_)
      {
        frontier_[index][0] = node;
      }
      // else if (node.g_ == frontier_[index][0].g_)
      // {
      //   if (node.parent_ < frontier_[index][0].parent_)
      //   {
      //     frontier_[index][0] = node;
      //   }
      // }
    }
  }
}

void TeamOrienteeringProblemSolver::AddToClosed(const Node& node)
{
  int index = GetNodeIndex(node);
  closed_[index] = true;
}

void TeamOrienteeringProblemSolver::AddToFrontier(const JointStateNode& node,
                                                  std::unordered_map<std::string, JointStateNode>& frontier)
{
  std::string node_label = node.GetLabel();
  auto it = frontier.find(node_label);
  if (it == frontier.end())
  {
    frontier.emplace(node_label, node);
    return;
  }

  if (it->second.g_ < node.g_)
  {
    it->second = node;
  }
}

void TeamOrienteeringProblemSolver::AddToClosed(const JointStateNode& node, std::unordered_set<std::string>& closed)
{
  std::string node_label = node.GetLabel();
  closed.insert(node_label);
}

bool TeamOrienteeringProblemSolver::CanBeVisited(int node_id)
{
  return node_set_.IsElementInSet(node_id);
}

bool CompareRewardEfficiency(const std::pair<double, int>& a, const std::pair<double, int>& b)
{
  return a.first > b.first;
}

double TeamOrienteeringProblemSolver::GetHeuristicValue(Node node)
{
  int curr_t = node.t_;
  int curr_node_id = node.id_;

  int remaining_t = time_interval_num_ - curr_t;

  if (heuristic_type_ == SIMPLE)
  {
    double max_profit = 0;
    // for (int node_id = 0; node_id < graph_node_num_; node_id++)
    // {
    //   // if (node_id != node.id_ && Visited(node.visited_nodes_, node_id))
    //   if (node_id != node.id_ && node.visited_nodes_.IsElementInSet(node_id))
    //   {
    //     continue;
    //   }

    //   if (distance_matrix_[curr_node_id][node_id] > remaining_t)
    //   {
    //     continue;
    //   }

    //   double remaining_max_reward = remaining_best_profits_[node_id][curr_t];
    //   max_profit = max_profit > remaining_max_reward ? max_profit : remaining_max_reward;
    // }
    // return max_profit * std::max(remaining_t, 0);

    return remaining_best_profits_all_nodes_[curr_t] * remaining_t;
  }
  else if (heuristic_type_ == KNAPSACK)
  {
    // Knapsack heuristic
    std::vector<std::pair<double, int>> reward_efficiency;
    for (int node_id = 0; node_id < graph_node_num_; node_id++)
    {
      // if (node_id != node.id_ && Visited(node.visited_nodes_, node_id))
      if (node_id != node.id_ && node.visited_nodes_.IsElementInSet(node_id))
      {
        continue;
      }

      if (distance_matrix_[curr_node_id][node_id] > remaining_t)
      {
        continue;
      }

      double remaining_reward_efficiency = remaining_best_profits_[node_id][curr_t];

      if (node_id != node.id_)
      {
        remaining_reward_efficiency /= distance_matrix_[curr_node_id][node_id];
      }

      reward_efficiency.emplace_back(remaining_reward_efficiency, node_id);
    }

    int remaining_node_num = reward_efficiency.size();
    if (remaining_node_num == 0)
    {
      return 0;
    }

    std::sort(reward_efficiency.begin(), reward_efficiency.end(), CompareRewardEfficiency);

    double max_profit = 0.0;
    int reward_efficiency_index = 0;
    int next_node_id = reward_efficiency[reward_efficiency_index].second;
    int next_t = distance_matrix_[node.id_][reward_efficiency[reward_efficiency_index].second];
    int used_t = next_t;
    bool add_last = true;
    while (used_t <= remaining_t)
    {
      max_profit += remaining_best_profits_[next_node_id][curr_t];
      reward_efficiency_index++;
      if (reward_efficiency_index >= remaining_node_num)
      {
        add_last = false;
        break;
      }
      next_node_id = reward_efficiency[reward_efficiency_index].second;
      next_t = distance_matrix_[node.id_][next_node_id];
      used_t += next_t;
    }
    if (add_last)
    {
      max_profit += remaining_best_profits_[next_node_id][curr_t];
    }
    return max_profit;
  }
  else
  {
    return 999.0 * std::max(remaining_t, 0);
  }
}

double TeamOrienteeringProblemSolver::SolveSingleVehicle(int start_graph_node_id, int goal_graph_node_id,
                                                         std::vector<int>& path)
{
  // TODO: If time_budget is larger than profits dimension, augment profits with zeros

  bool debug = false;
  bool debug_save_to_file = false;

  time_budget_ = std::min(time_budget_, time_interval_num_);

  frontier_.clear();
  closed_.clear();

  frontier_.resize(graph_node_num_ * time_interval_num_, std::vector<Node>());
  closed_.resize(graph_node_num_ * time_interval_num_, false);
  all_nodes_.clear();

  // Node start_node(start_node_id, 0, profits_[start_node_id][0], profits_[start_node_id][0], 0,
  //                 AddVisitedNodeID(0, start_node_id));
  Node start_node(start_graph_node_id, 0, profits_[start_graph_node_id][0], profits_[start_graph_node_id][0], 0,
                  graph_node_num_);
  start_node.visited_nodes_.Insert(start_graph_node_id);
  start_node.label_id_ = 0;

  double start_node_heuristic = GetHeuristicValue(start_node);
  start_node.f_ = start_node.g_ + start_node_heuristic;

  Node best_so_far_node;

  if (debug)
  {
    std::cout << "start node: ";
    PrintNode(start_node);
  }

  std::priority_queue<Node, std::vector<Node>, CompareNode> pq;
  pq.push(start_node);
  all_nodes_.push_back(start_node);

  /////DEBUG////////
  std::vector<std::vector<double>> all_vt;

  int iteration_count = 0;
  while (!pq.empty())
  {
    Node curr_node = pq.top();
    pq.pop();

    // if (iteration_count < 10)
    // {
    //   debug = true;
    // }
    // else
    // {
    //   debug = false;
    // }

    // if (curr_node.id_ == 78 && curr_node.t_ == 65)
    // {
    //   debug = true;
    // }
    // else
    // {
    //   debug = false;
    // }

    if (debug)
    {
      std::cout << "-----------Iteration " << iteration_count << std::endl;
      std::cout << "poping ";
      PrintNode(curr_node);
    }

    if (IsPruned(curr_node, best_so_far_node))
    {
      if (debug)
      {
        std::cout << "Pruned" << std::endl;
      }
      continue;
    }

    ///////DEBUG////////
    if (debug_save_to_file)
    {
      std::vector<double> vt;
      vt.push_back(curr_node.id_);
      vt.push_back(curr_node.t_);
      vt.push_back(curr_node.g_);
      vt.push_back(curr_node.parent_iteration_id_);
      all_vt.push_back(vt);
    }

    iteration_count++;

    if (debug)
    {
      std::cout << "Add to frontiers" << std::endl;
    }
    AddToFrontier(curr_node);

    if (use_closed_set_)
    {
      // k = 1, only keeping the best one
      if (debug)
      {
        std::cout << "Add to close" << std::endl;
      }
      AddToClosed(curr_node);
    }

    if (debug)
    {
      std::cout << "curr g: " << curr_node.g_ << std::endl;
      std::cout << "best g: " << best_so_far_node.g_ << std::endl;
    }

    if (best_so_far_node.id_ == -1 || best_so_far_node.g_ < curr_node.g_)
    {
      best_so_far_node = curr_node;
    }

    if (debug)
    {
      std::cout << "best so far node: ";
      PrintNode(best_so_far_node);
    }

    for (int next_graph_node_id = 0; next_graph_node_id < graph_node_num_; next_graph_node_id++)
    {
      Node next_node;
      next_node.id_ = next_graph_node_id;
      if (next_graph_node_id == curr_node.id_)
      {
        int next_t = curr_node.t_ + 1;
        if (next_t >= time_budget_)
        {
          continue;
        }
        next_node.t_ = next_t;
        if (accumulate_reward_at_same_node_)
        {
          next_node.g_ = curr_node.g_ + profits_[curr_node.id_][next_t];
        }
        else
        {
          next_node.g_ = curr_node.g_;
        }
        next_node.f_ = curr_node.g_ + GetHeuristicValue(next_node);
        next_node.visited_nodes_ = curr_node.visited_nodes_;
      }
      // else if (!CanBeVisited(next_graph_node_id) || Visited(curr_node.visited_nodes_, next_graph_node_id))
      // {
      //   continue;
      // }
      // else if (Visited(curr_node.visited_nodes_, next_graph_node_id))
      else if (!CanBeVisited(next_graph_node_id) || curr_node.visited_nodes_.IsElementInSet(next_graph_node_id))
      {
        continue;
      }
      else
      {
        int next_t = curr_node.t_ + distance_matrix_[curr_node.id_][next_node.id_];
        if (next_t >= time_budget_)
        {
          continue;
        }
        next_node.t_ = next_t;
        next_node.g_ = curr_node.g_ + profits_[next_graph_node_id][next_t];
        // next_node.visited_nodes_ = AddVisitedNodeID(curr_node.visited_nodes_, next_graph_node_id);
        next_node.visited_nodes_ = curr_node.visited_nodes_;
        next_node.visited_nodes_.Insert(next_graph_node_id);
      }
      next_node.f_ = next_node.g_ + GetHeuristicValue(next_node);

      // if (next_node.id_ == 29 && next_node.t_ == 97 || curr_node.id_ == 78 && curr_node.t_ == 88)
      // if (next_node.id_ == 78 && next_node.t_ == 65)
      // {
      //   debug = true;
      //   std::cout << "poping curr node: ";
      //   PrintNode(curr_node);
      // }
      // else
      // {
      //   debug = false;
      // }

      if (debug)
      {
        std::cout << "next node: ";
        PrintNode(next_node);
      }

      if (IsPruned(next_node, best_so_far_node))
      {
        if (debug)
        {
          std::cout << "Pruned " << std::endl;
        }

        continue;
      }
      else
      {
        next_node.parent_ = curr_node.label_id_;
        next_node.parent_iteration_id_ = iteration_count;
        next_node.label_id_ = all_nodes_.size();
        pq.push(next_node);
        all_nodes_.push_back(next_node);

        if (debug)
        {
          std::cout << "Pushing " << std::endl;
          std::cout << "add to fontiers2" << std::endl;
        }
        AddToFrontier(next_node);
      }
    }
  }

  node_expanded_count_ = iteration_count;

  int curr_node_label_id = best_so_far_node.label_id_;

  // std::cout << "best node t: " << best_so_far_node.t_ << std::endl;

  std::vector<int> node_indices;

  while (curr_node_label_id != -1)
  {
    int node_id = all_nodes_[curr_node_label_id].id_;
    int t = all_nodes_[curr_node_label_id].t_;
    node_indices.push_back(node_id);
    curr_node_label_id = all_nodes_[curr_node_label_id].parent_;
  }

  std::reverse(node_indices.begin(), node_indices.end());

  path = node_indices;

  ///////DEBUG////////
  if (debug_save_to_file)
  {
    std::string filename = "/home/caochao/Work/ros_ws/tare_system/src/tare_planner/data/ours_vt.txt";
    misc_utils_ns::Save2DVectorToFile<double>(all_vt, filename);
  }

  return best_so_far_node.g_;
}

void TeamOrienteeringProblemSolver::ProcessJointStateNodeNeighbors(
    const JointStateNode& curr_joint_state_node, const JointStateNode& best_so_far_joint_state_node,
    std::unordered_map<std::string, JointStateNode>& frontier, std::unordered_set<std::string>& closed,
    std::priority_queue<JointStateNode, std::vector<JointStateNode>, CompareJointStateNode>& pq,
    std::vector<JointStateNode>& all_nodes, int time_budget, int K, bool debug)
{
  // Find the node with the smallest t
  int min_t_vehicle_index = 0;
  int min_t = INT_MAX;
  for (int i = 0; i < vehicle_num_; i++)
  {
    if (best_so_far_joint_state_node.nodes_[i].t_ < min_t)
    {
      min_t = best_so_far_joint_state_node.nodes_[i].t_;
      min_t_vehicle_index = i;
    }
  }

  if (debug)
  {
    std::cout << "min t index: " << min_t_vehicle_index << " min t: " << min_t << std::endl;
  }

  std::priority_queue<Node, std::vector<Node>, CompareNode> candidate_nodes_pq;

  Node curr_node = curr_joint_state_node.nodes_[min_t_vehicle_index];

  // Generate next node candidates
  for (int next_graph_node_id = 0; next_graph_node_id < graph_node_num_; next_graph_node_id++)
  {
    Node next_node;
    next_node.id_ = next_graph_node_id;
    if (next_graph_node_id == curr_node.id_)
    {
      int next_t = curr_node.t_ + 1;
      if (next_t >= time_budget)
      {
        continue;
      }
      next_node.t_ = next_t;
      if (accumulate_reward_at_same_node_)
      {
        next_node.g_ = curr_node.g_ + profits_[curr_node.id_][next_t];
      }
      else
      {
        next_node.g_ = curr_node.g_;
      }
      next_node.f_ = curr_node.g_ + GetHeuristicValue(next_node);
      next_node.visited_nodes_ = curr_node.visited_nodes_;
    }
    else
    {
      bool visited = false;
      for (const auto& node : curr_joint_state_node.nodes_)
      {
        // if (next_graph_node_id != start_node_s_id_ && node.visited_nodes_.IsElementInSet(next_graph_node_id))
        if (node.visited_nodes_.IsElementInSet(next_graph_node_id))
        {
          visited = true;
          break;
        }
      }
      if (visited)
      {
        continue;
      }

      int next_t = curr_node.t_ + distance_matrix_[curr_node.id_][next_node.id_];
      if (next_t >= time_budget)
      {
        continue;
      }
      next_node.t_ = next_t;
      next_node.g_ = curr_node.g_ + profits_[next_graph_node_id][next_t];
      next_node.visited_nodes_ = curr_node.visited_nodes_;
      next_node.visited_nodes_.Insert(next_graph_node_id);
    }
    next_node.f_ = next_node.g_ + GetHeuristicValue(next_node);
    candidate_nodes_pq.push(next_node);

    if (debug)
    {
      PrintNode(next_node);
      std::cout << "Pushing to candidate pq" << std::endl;
    }
  }

  // Pop K nodes
  std::vector<Node> selected_nodes;
  int pop_num = std::min(K, (int)(candidate_nodes_pq.size()));
  for (int i = 0; i < pop_num; i++)
  {
    Node node = candidate_nodes_pq.top();
    selected_nodes.push_back(node);
    candidate_nodes_pq.pop();
  }

  if (debug)
  {
    std::cout << "Selected nodes: " << std::endl;
  }
  for (int i = 0; i < selected_nodes.size(); i++)
  {
    JointStateNode new_joint_state_node = curr_joint_state_node;
    new_joint_state_node.SetNode(min_t_vehicle_index, selected_nodes[i]);

    if (!IsPruned(new_joint_state_node, best_so_far_joint_state_node, frontier, closed))
    {
      new_joint_state_node.parent_ = curr_joint_state_node.label_id_;
      new_joint_state_node.label_id_ = all_nodes.size();
      new_joint_state_node.move_vehicle_id_ = min_t_vehicle_index;
      pq.push(new_joint_state_node);
      all_nodes.push_back(new_joint_state_node);
      if (debug)
      {
        std::cout << "i: " << i << std::endl;
        PrintNode(new_joint_state_node);
        std::cout << "pushed to pq" << std::endl;
      }
    }
  }
  if (debug)
  {
    std::cout << "Add the remaining nodes back to pq with discounted priority" << std::endl;
  }

  std::vector<Node> remaining_nodes;
  double alpha = 0.5;
  while (!candidate_nodes_pq.empty())
  {
    Node node = candidate_nodes_pq.top();
    remaining_nodes.push_back(node);
    candidate_nodes_pq.pop();
  }

  for (int i = 0; i < remaining_nodes.size(); i++)
  {
    JointStateNode new_joint_state_node = curr_joint_state_node;
    new_joint_state_node.SetNode(min_t_vehicle_index, remaining_nodes[i]);
    new_joint_state_node.DiscountF(alpha);

    if (!IsPruned(new_joint_state_node, best_so_far_joint_state_node, frontier, closed))
    {
      new_joint_state_node.parent_ = curr_joint_state_node.label_id_;
      new_joint_state_node.label_id_ = all_nodes.size();
      new_joint_state_node.move_vehicle_id_ = min_t_vehicle_index;
      pq.push(new_joint_state_node);
      all_nodes.push_back(new_joint_state_node);
      if (debug)
      {
        std::cout << "i: " << i << std::endl;
        PrintNode(new_joint_state_node);
      }
      if (debug)
      {
        std::cout << "pushed to pq" << std::endl;
      }
    }
  }
}

double TeamOrienteeringProblemSolver::SolveMultiVehicle(int start_node_id, int end_goal_id,
                                                        std::vector<std::vector<int>>& paths, int K)
{
  bool debug = true;
  bool debug_save_to_file = true;

  start_node_s_id_ = start_node_id;

  time_budget_ = std::min(time_budget_, time_interval_num_);

  std::unordered_map<std::string, JointStateNode> frontier;
  std::unordered_set<std::string> closed;
  std::vector<JointStateNode> all_nodes;

  Node start_node(start_node_id, 0, profits_[start_node_id][0], profits_[start_node_id][0], 0, graph_node_num_);
  start_node.visited_nodes_.Insert(start_node_id);

  double start_node_heuristic = GetHeuristicValue(start_node);
  start_node.f_ = start_node.g_ + start_node_heuristic;

  JointStateNode joint_state_start_node(vehicle_num_);
  for (int i = 0; i < vehicle_num_; i++)
  {
    joint_state_start_node.SetNode(i, start_node);
  }
  joint_state_start_node.label_id_ = all_nodes.size();

  frontier.emplace(joint_state_start_node.GetLabel(), joint_state_start_node);

  JointStateNode best_so_far_joint_state_node = joint_state_start_node;

  if (debug)
  {
    std::cout << "start node: " << std::endl;
    PrintNode(joint_state_start_node);

    std::cout << "runtime limit: " << runtime_limit_ << std::endl;
  }

  std::priority_queue<JointStateNode, std::vector<JointStateNode>, CompareJointStateNode> pq;
  pq.push(joint_state_start_node);
  all_nodes.push_back(joint_state_start_node);

  int iteration_count = 0;

  misc_utils_ns::Timer runtime("runtime");
  runtime.Start();
  // auto start_time = clock();
  int tick = 1;

  std::vector<std::vector<double>> all_ft;

  while (!pq.empty())
  {
    if (iteration_count < 10)
    {
      debug = true;
    }
    else
    {
      debug = false;
    }

    if (debug)
    {
      std::cout << "-----------Iteration " << iteration_count << std::endl;
    }

    int runtime_ms = runtime.Split("ms");
    // int runtime_ms = runtime.GetDuration("ms");
    // std::cout << "runtime ms: " << runtime_ms << std::endl;

    // auto elapsed_time = (double)(clock() - start_time) / CLOCKS_PER_SEC;

    if (runtime_ms > runtime_limit_)
    {
      std::cout << "runtime limit " << runtime_limit_ << " ms reached" << std::endl;
      std::cout << "best so far node: ";
      PrintNode(best_so_far_joint_state_node);
      break;
    }
    else if (debug)
    {
      std::cout << "runtime: " << runtime_ms << std::endl;
    }
    else if (runtime_ms > tick * 1000)
    {
      std::cout << "runtime: " << runtime_ms << " best g: " << best_so_far_joint_state_node.g_
                << " queue size: " << pq.size() << std::endl;
      tick += 1;

      if (debug_save_to_file)
      {
        std::vector<double> ft;
        ft.push_back(runtime_ms);
        ft.push_back(best_so_far_joint_state_node.g_);
        all_ft.push_back(ft);
      }
    }

    JointStateNode curr_node = pq.top();
    pq.pop();

    if (debug)
    {
      std::cout << "poping " << std::endl;
      PrintNode(curr_node);
    }

    if (IsPruned(curr_node, best_so_far_joint_state_node, frontier, closed))
    {
      if (debug)
      {
        std::cout << "Pruned" << std::endl;
      }
      continue;
    }

    iteration_count++;

    if (debug)
    {
      std::cout << "Add to frontiers" << std::endl;
    }
    AddToFrontier(curr_node, frontier);

    if (use_closed_set_)
    {
      // k = 1, only keeping the best one
      if (debug)
      {
        std::cout << "Add to close" << std::endl;
      }
      AddToClosed(curr_node, closed);
    }

    if (debug)
    {
      std::cout << "curr g: " << curr_node.g_ << std::endl;
      std::cout << "best g: " << best_so_far_joint_state_node.g_ << std::endl;
    }

    // if (best_so_far_joint_state_node.parent_iteration_id_ == -1 || best_so_far_joint_state_node.g_ < curr_node.g_)
    if (best_so_far_joint_state_node.g_ < curr_node.g_)
    {
      best_so_far_joint_state_node = curr_node;
    }

    if (debug)
    {
      std::cout << "best so far node: " << std::endl;
      PrintNode(best_so_far_joint_state_node);
    }

    ProcessJointStateNodeNeighbors(curr_node, best_so_far_joint_state_node, frontier, closed, pq, all_nodes,
                                   time_budget_, K, debug);
  }

  node_expanded_count_ = iteration_count;

  int curr_node_label_id = best_so_far_joint_state_node.label_id_;
  std::vector<std::vector<int>> node_indices(vehicle_num_, std::vector<int>());
  std::vector<int> latest_time(vehicle_num_, -1);

  std::cout << "Finished: best node: " << std::endl;
  PrintNode(best_so_far_joint_state_node);

  // std::cout << "distance matrix: " << std::endl;
  // for (int i = 0; i < distance_matrix_.size(); i++)
  // {
  //   std::cout << i << ": ";
  //   for (int j = 0; j < distance_matrix_[i].size(); j++)
  //   {
  //     std::cout << distance_matrix_[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }

  std::cout << "all node num: " << all_nodes.size() << std::endl;
  std::cout << "node indices size: " << node_indices.size() << std::endl;

  // std::cout << "back tracking node indices: " << std::endl;
  while (curr_node_label_id != -1)
  {
    for (int i = 0; i < vehicle_num_; i++)
    {
      int node_id = all_nodes[curr_node_label_id].nodes_[i].id_;
      int t = all_nodes[curr_node_label_id].nodes_[i].t_;
      if (t != latest_time[i])
      {
        node_indices[i].push_back(node_id);
        latest_time[i] = t;
      }
      // std::cout << "v" << i << ": " << node_id << " " << t << " ";
    }
    // std::cout << std::endl;
    curr_node_label_id = all_nodes[curr_node_label_id].parent_;
  }

  std::cout << "node indices: " << std::endl;
  for (int i = 0; i < node_indices.size(); i++)
  {
    std::cout << i << ": ";
    for (int j = 0; j < node_indices[i].size(); j++)
    {
      std::cout << node_indices[i][j] << " ";
    }
    std::cout << std::endl;
  }

  for (int i = 0; i < vehicle_num_; i++)
  {
    std::reverse(node_indices[i].begin(), node_indices[i].end());
  }

  paths = node_indices;

  if (debug_save_to_file)
  {
    std::string filename =
        "/home/caochao/Work/ros_ws/tare_system/src/tare_planner/data/ours_ft_" + std::to_string(K) + ".txt";
    misc_utils_ns::Save2DVectorToFile<double>(all_ft, filename);
  }

  return best_so_far_joint_state_node.g_;
}

double TeamOrienteeringProblemSolver::SolveMultiVehicleGreedy(const std::vector<int>& start_node_ids,
                                                              std::vector<int>& end_goal_ids,
                                                              std::vector<std::vector<int>>& paths)
{
  std::cout << "solving greedily" << std::endl;
  paths.clear();
  IncludeAllNodes();

  double total_profit = 0;

  for (int vehicle_id = 0; vehicle_id < vehicle_num_; vehicle_id++)
  {
    std::vector<int> single_robot_path;
    if (vehicle_id > 0)
    {
      // std::cout << "before excluding: ";
      // node_set_.Print();
      // std::cout << "Excluding ";
      // for (const auto& node_id : paths[vehicle_id - 1])
      // {
      //   std::cout << node_id << " ";
      // }
      // std::cout << std::endl;
      ExcludeNodes(paths[vehicle_id - 1]);
      // std::cout << "after excluding: ";
      // node_set_.Print();
    }
    double max_profit = SolveSingleVehicle(start_node_ids[vehicle_id], 0, single_robot_path);

    // std::cout << "vehicle " << vehicle_id << " max profit: " << max_profit << std::endl;
    // std::cout << "path: ";
    // for (int i = 0; i < single_robot_path.size(); i++)
    // {
    //   std::cout << single_robot_path[i] << " ";
    // }
    // std::cout << std::endl;

    paths.push_back(single_robot_path);

    total_profit += max_profit;
  }

  return total_profit;
}

void TeamOrienteeringProblemSolver::GeneratePBSRootNode(PBS_ns::PBS& pbs)
{
  // std::cout << "generating root node" << std::endl;
  auto root = new PBS_ns::PBSNode();
  root->profit_ = 0.0;
  root->paths_.clear();
  pbs.paths_.clear();

  std::set<int> higher_agents;
  for (auto i = 0; i < vehicle_num_; i++)
  {
    // std::cout << "solving path for vehicle " << i << std::endl;
    IncludeAllNodes();
    for (int j = 0; j < vehicle_num_; j++)
    {
      if (i != j)
      {
        ExcludeNode(start_graph_node_ids_[j]);
      }
    }
    std::vector<int> single_vehicle_path;
    double single_vehicle_profit =
        SolveSingleVehicle(start_graph_node_ids_[i], goal_graph_node_ids_[i], single_vehicle_path);
    // std::cout << "path: " << std::endl;
    // for (auto node_id : single_vehicle_path)
    // {
    //   std::cout << node_id << " ";
    // }
    // std::cout << std::endl;
    if (single_vehicle_path.empty())
    {
      std::cout << "No path exists for vehicle " << i << std::endl;
      return;
    }
    root->paths_.emplace_back(i, single_vehicle_path);
    pbs.paths_.push_back(single_vehicle_path);
    root->profit_ += single_vehicle_profit;
    root->profits_.push_back(single_vehicle_profit);
  }
  root->depth = 0;
  for (int a1 = 0; a1 < vehicle_num_; a1++)
  {
    for (int a2 = a1 + 1; a2 < vehicle_num_; a2++)
    {
      if (pbs.hasConflicts(a1, a2))
      {
        root->conflicts_.emplace_back(new PBS_ns::Conflict(a1, a2));
      }
    }
  }

  pbs.pushNode(root);

  // std::cout << std::endl << "Root node: " << std::endl;
  // PrintNode(*root);
}

bool TeamOrienteeringProblemSolver::FindPathForSingleAgent(PBS_ns::PBS& pbs, PBS_ns::PBSNode& node,
                                                           const std::set<int>& higher_agents, int a,
                                                           std::vector<int>& new_path)
{
  // std::cout << "Find path for agent " << a << std::endl;
  new_path.clear();
  // Reset exclusion
  IncludeAllNodes();
  // std::cout << "Included nodes: " << std::endl;
  // PrintIncludedNodes();
  for (const auto& higher_agent_id : higher_agents)
  {
    // std::cout << "excluding agent: " << higher_agent_id << " " << std::endl;
    // for (const auto& node_id : pbs.paths_[higher_agent_id])
    // {
    //   std::cout << node_id << " ";
    // }
    // std::cout << std::endl;
    ExcludeNodes(pbs.paths_[higher_agent_id]);
    // std::cout << "Included nodes: " << std::endl;
    // PrintIncludedNodes();
  }
  double single_vehicle_profit = SolveSingleVehicle(start_graph_node_ids_[a], goal_graph_node_ids_[a], new_path);

  // std::cout << "new path for agent " << a << ":" << std::endl;
  // for (const auto& node_id : new_path)
  // {
  //   std::cout << node_id << " ";
  // }
  // std::cout << std::endl;

  if (new_path.empty())
  {
    return false;
  }

  pbs.paths_[a] = new_path;
  node.paths_.emplace_back(a, new_path);
  node.profit_ = node.profit_ - node.profits_[a] + single_vehicle_profit;
  node.profits_[a] = single_vehicle_profit;

  return true;
}

void TeamOrienteeringProblemSolver::GeneratePBSChildNode(PBS_ns::PBS& pbs, int child_id, PBS_ns::PBSNode* parent,
                                                         int low, int high)
{
  bool debug = false;

  MY_ASSERT(child_id == 0 || child_id == 1);
  parent->children[child_id] = new PBS_ns::PBSNode(*parent);
  auto node = parent->children[child_id];
  node->constraint_.set(low, high);
  pbs.priority_graph_[high][low] = false;
  pbs.priority_graph_[low][high] = true;

  pbs.topologicalSort(pbs.ordered_agents_);
  // std::cout << "topo sorted agents: " << std::endl;
  // for (const auto& agent_id : pbs.ordered_agents_)
  // {
  //   std::cout << agent_id << " ";
  // }
  // std::cout << std::endl;

  std::vector<int> topological_orders(vehicle_num_);  // map agent i to its position in ordered_agents
  auto i = vehicle_num_ - 1;
  for (const auto& a : pbs.ordered_agents_)
  {
    topological_orders[a] = i;
    i--;
  }

  std::priority_queue<std::pair<int, int>> to_replan;  // <position in ordered_agents, agent id>
  std::vector<bool> lookup_table(vehicle_num_, false);
  to_replan.emplace(topological_orders[low], low);
  lookup_table[low] = true;

  {  // find conflicts where one agent is higher than high and the other agent is lower than low
    std::set<int> higher_agents;
    auto p = pbs.ordered_agents_.rbegin();
    std::advance(p, topological_orders[high]);
    assert(*p == high);
    pbs.getHigherPriorityAgents(p, higher_agents);
    higher_agents.insert(high);

    std::set<int> lower_agents;
    auto p2 = pbs.ordered_agents_.begin();
    std::advance(p2, vehicle_num_ - 1 - topological_orders[low]);
    assert(*p2 == low);
    pbs.getLowerPriorityAgents(p2, lower_agents);

    for (const auto& conflict : node->conflicts_)
    {
      int a1 = conflict->a1;
      int a2 = conflict->a2;
      if (a1 == low or a2 == low)
        continue;
      if (topological_orders[a1] > topological_orders[a2])
      {
        std::swap(a1, a2);
      }
      if (!lookup_table[a1] and lower_agents.find(a1) != lower_agents.end() and
          higher_agents.find(a2) != higher_agents.end())
      {
        to_replan.emplace(topological_orders[a1], a1);
        lookup_table[a1] = true;
      }
    }
  }

  while (!to_replan.empty())
  {
    int a, rank;
    std::tie(rank, a) = to_replan.top();
    to_replan.pop();
    lookup_table[a] = false;
    if (debug)
    {
      std::cout << "Replan agent " << a << std::endl;
    }
    // Re-plan path
    std::set<int> higher_agents;
    auto p = pbs.ordered_agents_.rbegin();
    std::advance(p, rank);
    MY_ASSERT(*p == a);
    pbs.getHigherPriorityAgents(p, higher_agents);
    MY_ASSERT(!higher_agents.empty());
    if (debug)
    {
      std::cout << "Higher agents: ";
      for (auto i : higher_agents)
        std::cout << i << ",";
      std::cout << std::endl;
    }
    std::vector<int> new_path;
    if (!FindPathForSingleAgent(pbs, *node, higher_agents, a, new_path))
    {
      delete node;
      parent->children[child_id] = nullptr;
      std::cout << "Cannot find path for vehicle " << a << std::endl;
      return;
    }

    // std::cout << "new profit: " << node->profit_ << std::endl;

    // Delete old conflicts
    // std::cout << "Deleting old conflicts" << std::endl;
    for (auto c = node->conflicts_.begin(); c != node->conflicts_.end();)
    {
      if ((*c)->a1 == a or (*c)->a2 == a)
        c = node->conflicts_.erase(c);
      else
        ++c;
    }

    // std::cout << "after deletion: " << std::endl;
    // for (auto c : node->conflicts_)
    // {
    //   std::cout << *c << std::endl;
    // }

    // Update conflicts and to_replan
    // std::cout << "Updating conflicts and replan" << std::endl;
    std::set<int> lower_agents;
    auto p2 = pbs.ordered_agents_.begin();
    std::advance(p2, vehicle_num_ - 1 - rank);
    MY_ASSERT(*p2 == a);
    pbs.getLowerPriorityAgents(p2, lower_agents);
    // if (debug and !lower_agents.empty())
    // {
    //   std::cout << "Lower agents: ";
    //   for (auto i : lower_agents)
    //     std::cout << i << ",";
    //   std::cout << std::endl;
    // }

    // std::cout << "updated paths: " << std::endl;
    // for (int i = 0; i < pbs.paths_.size(); i++)
    // {
    //   std::cout << i << ": ";
    //   for (int j = 0; j < pbs.paths_[i].size(); j++)
    //   {
    //     if (j > 0 && pbs.paths_[i][j] != pbs.paths_[i][j - 1])
    //     {
    //       std::cout << pbs.paths_[i][j] << " ";
    //     }
    //   }
    //   std::cout << std::endl;
    // }

    // Find new conflicts
    // std::cout << "Finding new conflicts" << std::endl;
    for (auto a2 = 0; a2 < vehicle_num_; a2++)
    {
      if (a2 == a or lookup_table[a2] or higher_agents.count(a2) > 0)  // already in to_replan or has higher priority
        continue;

      bool print_debug = false;
      if (pbs.hasConflicts(a, a2))
      {
        node->conflicts_.emplace_back(new PBS_ns::Conflict(a, a2));
        if (lower_agents.count(a2) > 0)  // has a collision with a lower priority agent
        {
          if (debug)
            std::cout << "\t" << a2 << " needs to be replanned due to collisions with " << a << std::endl;
          to_replan.emplace(topological_orders[a2], a2);
          lookup_table[a2] = true;
        }
      }
    }
    // std::cout << "after adding: " << std::endl;
    // for (auto c : node->conflicts_)
    // {
    //   std::cout << *c << std::endl;
    // }
  }
}
void TeamOrienteeringProblemSolver::PushPBSNodes(PBS_ns::PBS& pbs, PBS_ns::PBSNode* n1, PBS_ns::PBSNode* n2)
{
  if (n1 != nullptr and n2 != nullptr)
  {
    if (n1->profit_ > n2->profit_)
    {
      pbs.pushNode(n2);
      pbs.pushNode(n1);
    }
    else
    {
      pbs.pushNode(n1);
      pbs.pushNode(n2);
    }
  }
  else if (n1 != nullptr)
  {
    pbs.pushNode(n1);
  }
  else if (n2 != nullptr)
  {
    pbs.pushNode(n2);
  }
}

double TeamOrienteeringProblemSolver::SolveMultiVehiclePBS(std::vector<std::vector<int>>& paths)
{
  bool debug = false;
  if (debug)
  {
    std::cout << "solving with Priority-based Search" << std::endl;
  }
  paths.clear();

  double total_profit = 0;

  PBS_ns::PBS pbs(vehicle_num_);

  GeneratePBSRootNode(pbs);

  int iteration = 0;
  while (!pbs.open_list_empty())
  {
    // std::cout << "-------------iteration: " << iteration++ << std::endl;
    auto curr = pbs.selectNode();

    if (pbs.terminate(curr))
    {
      total_profit = curr->profit_;
      paths = pbs.paths_;
      break;
    }

    curr->conflict_ = pbs.chooseConflict(*curr);

    if (debug)
    {
      std::cout << "	Expand " << *curr << "	on " << *(curr->conflict_) << std::endl;
      std::cout << "curr paths: " << std::endl;
      for (int i = 0; i < pbs.paths_.size(); i++)
      {
        std::cout << i << ": ";
        for (int j = 0; j < pbs.paths_[i].size(); j++)
        {
          if (j > 0 && pbs.paths_[i][j] != pbs.paths_[i][j - 1])
          {
            std::cout << pbs.paths_[i][j] << " ";
          }
        }
        std::cout << std::endl;
      }
    }

    MY_ASSERT(!pbs.hasHigherPriority(curr->conflict_->a1, curr->conflict_->a2) &&
              !pbs.hasHigherPriority(curr->conflict_->a2, curr->conflict_->a1));
    std::vector<std::vector<int>> copy(pbs.paths_);
    // std::cout << "---Generating child 0" << std::endl;
    GeneratePBSChildNode(pbs, 0, curr, curr->conflict_->a1, curr->conflict_->a2);
    pbs.paths_ = copy;
    // std::cout << "---Generating child 1" << std::endl;
    GeneratePBSChildNode(pbs, 1, curr, curr->conflict_->a2, curr->conflict_->a1);
    PushPBSNodes(pbs, curr->children[0], curr->children[1]);
    curr->clear();
  }
  return total_profit;
}

void TeamOrienteeringProblemSolver::ComputeCBSNodeHeuristic(CBS_ns::CBSNode& node)
{
  node.h_ = node.profit_;
}

void TeamOrienteeringProblemSolver::GetConflictingNodes(CBS_ns::CBSNode* node, int a1, int a2,
                                                        std::vector<int>& conflicting_nodes)
{
  conflicting_nodes.clear();
  misc_utils_ns::GetCommonElements(node->paths_[a1], node->paths_[a2], conflicting_nodes);
  misc_utils_ns::UniquifyIntVector(conflicting_nodes);
}

void TeamOrienteeringProblemSolver::FindConflicts(CBS_ns::CBSNode& node)
{
  node.conflicts_.clear();
  for (int a1 = 0; a1 < vehicle_num_; a1++)
  {
    for (int a2 = a1 + 1; a2 < vehicle_num_; a2++)
    {
      if (node.paths_[a1].empty() || node.paths_[a2].empty())
      {
        continue;
      }
      std::vector<int> conflicting_nodes;
      GetConflictingNodes(&node, a1, a2, conflicting_nodes);
      if (conflicting_nodes.empty())
      {
        continue;
      }
      for (auto node_id : conflicting_nodes)
      {
        node.conflicts_.emplace_back(new CBS_ns::Conflict(a1, a2, node_id));
      }
    }
  }
}

void TeamOrienteeringProblemSolver::GenerateCBSRootNode(CBS_ns::CBS& cbs)
{
  auto root = new CBS_ns::CBSNode();

  root->profit_ = 0.0;
  root->g_ = 0.0;

  root->paths_.clear();
  cbs.paths_.clear();

  for (auto i = 0; i < vehicle_num_; i++)
  {
    IncludeAllNodes();
    for (int j = 0; j < vehicle_num_; j++)
    {
      if (i != j)
      {
        ExcludeNode(start_graph_node_ids_[j]);
      }
    }
    std::vector<int> single_vehicle_path;
    double single_vehicle_profit =
        SolveSingleVehicle(start_graph_node_ids_[i], goal_graph_node_ids_[i], single_vehicle_path);
    if (single_vehicle_path.empty())
    {
      std::cout << "No path exists for vehicle " << i << std::endl;
      return;
    }
    root->paths_.push_back(single_vehicle_path);
    cbs.paths_.push_back(single_vehicle_path);
    root->profit_ += single_vehicle_profit;
    root->profits_.push_back(single_vehicle_profit);
  }

  root->depth = 0;

  FindConflicts(*root);

  root->h_ = 0.0;
  root->depth_ = 0;
  // TODO
  ComputeCBSNodeHeuristic(*root);
  cbs.pushNode(root);

  std::cout << std::endl << "Root node: " << std::endl;
  PrintNode(*root);
}

void TeamOrienteeringProblemSolver::GetInvalidAgentsAndExcludingNodes(const CBS_ns::CBSNode& node,
                                                                      std::vector<int>& invalid_agents,
                                                                      std::vector<std::vector<int>>& excluding_nodes)
{
  invalid_agents.clear();
  excluding_nodes.clear();
  excluding_nodes.resize(vehicle_num_);
  std::cout << "constraints: " << std::endl;
  for (const auto& constraint : node.constraints_)
  {
    std::cout << std::get<0>(constraint) << " / " << std::get<1>(constraint) << std::endl;
    int vehicle_id = std::get<0>(constraint);
    int node_id = std::get<1>(constraint);
    excluding_nodes[vehicle_id].push_back(node_id);
    if (node.paths_[vehicle_id].empty())
    {
      continue;
    }
    if (misc_utils_ns::ElementExistsInVector(node.paths_[vehicle_id], node_id))
    {
      invalid_agents.push_back(vehicle_id);
    }
  }
  // Sort and remove duplicates
  std::sort(invalid_agents.begin(), invalid_agents.end());
  invalid_agents.erase(std::unique(invalid_agents.begin(), invalid_agents.end()), invalid_agents.end());
}

void TeamOrienteeringProblemSolver::GenerateCBSChildNode(CBS_ns::CBSNode* node, CBS_ns::CBSNode* parent)
{
  std::vector<int> invalid_agents;
  std::vector<std::vector<int>> excluding_nodes;
  GetInvalidAgentsAndExcludingNodes(*node, invalid_agents, excluding_nodes);
  std::cout << "invalid agents: " << std::endl;
  for (const auto& agent : invalid_agents)
  {
    std::cout << agent << " ";
  }
  std::cout << std::endl;
  std::cout << "excluding nodes: " << std::endl;
  for (int i = 0; i < excluding_nodes.size(); i++)
  {
    std::cout << "v" << i << ": ";
    for (const auto& node_id : excluding_nodes[i])
    {
      std::cout << node_id << " ";
    }
    std::cout << std::endl;
  }
  MY_ASSERT(!invalid_agents.empty());
  for (auto agent : invalid_agents)
  {
    std::cout << "finding new path for agent " << agent << std::endl;
    FindPathForSingleAgent(*node, agent, excluding_nodes);
  }
  std::cout << "finding conflicts" << std::endl;
  FindConflicts(*node);
  ComputeCBSNodeHeuristic(*node);
  std::cout << "done generating child" << std::endl;
}
void TeamOrienteeringProblemSolver::PushCBSNodes(CBS_ns::CBS& cbs, CBS_ns::CBSNode* n1, CBS_ns::CBSNode* n2)
{
  if (n1 != nullptr)
  {
    cbs.pushNode(n1);
  }
  if (n2 != nullptr)
  {
    cbs.pushNode(n2);
  }
}

bool TeamOrienteeringProblemSolver::FindPathForSingleAgent(CBS_ns::CBSNode& node, int vehicle_id,
                                                           const std::vector<std::vector<int>>& excluding_nodes)
{
  // std::cout << "Find path for agent " << vehicle_id << std::endl;
  // Reset exclusion
  IncludeAllNodes();
  // std::cout << "Included nodes: " << std::endl;
  // PrintIncludedNodes();
  ExcludeNodes(excluding_nodes[vehicle_id]);

  // Excluding the start nodes of other vehicles
  for (int i = 0; i < start_graph_node_ids_.size(); i++)
  {
    if (i != vehicle_id)
    {
      ExcludeNode(start_graph_node_ids_[i]);
    }
  }

  // PrintIncludedNodes();
  double single_vehicle_profit =
      SolveSingleVehicle(start_graph_node_ids_[vehicle_id], goal_graph_node_ids_[vehicle_id], node.paths_[vehicle_id]);

  // std::cout << "new path for agent " << vehicle_id << ":" << std::endl;
  // for (const auto& node_id : node.paths_[vehicle_id])
  // {
  //   std::cout << node_id << " ";
  // }
  // std::cout << std::endl;

  if (node.paths_[vehicle_id].empty())
  {
    return false;
  }

  node.profit_ = node.profit_ - node.profits_[vehicle_id] + single_vehicle_profit;
  node.profits_[vehicle_id] = single_vehicle_profit;

  return true;
}

double TeamOrienteeringProblemSolver::SolveMultiVehicleCBS(std::vector<std::vector<int>>& paths)
{
  std::cout << "solving with Conflict-based Search" << std::endl;
  paths.clear();

  double total_profit = 0;

  CBS_ns::CBS cbs(vehicle_num_);

  std::cout << "generating root node " << std::endl;
  GenerateCBSRootNode(cbs);
  std::cout << "generated root node " << std::endl;

  bool debug = true;
  int iteration = 0;
  while (!cbs.open_list_empty())
  {
    auto curr = cbs.selectNode();

    if (debug)
    {
      std::cout << "-------------iteration: " << iteration++ << std::endl;
      // if (iteration > 10)
      // {
      //   break;
      // }
      std::cout << "selected current node: " << std::endl;
      PrintNode(*curr);
    }

    if (cbs.terminate(curr))
    {
      total_profit = curr->profit_;
      paths = cbs.paths_;
      break;
    }

    std::cout << "dbg created children" << std::endl;
    CBS_ns::CBSNode* child[2] = { new CBS_ns::CBSNode(*curr), new CBS_ns::CBSNode(*curr) };
    std::cout << "dbg choosing conflict" << std::endl;
    curr->conflict_ = cbs.chooseConflict(*curr);
    std::cout << "dgbg chosen conflict" << *(curr->conflict_) << std::endl;

    // Add constraints
    std::cout << "dbg adding constraints" << std::endl;
    child[0]->constraints_.push_back(curr->conflict_->constraint1);
    child[1]->constraints_.push_back(curr->conflict_->constraint2);

    std::vector<std::vector<int>> copy(cbs.paths_);
    std::cout << "---Generating child 0" << std::endl;
    GenerateCBSChildNode(child[0], curr);
    PrintNode(*child[0]);
    cbs.paths_ = copy;
    std::cout << "---Generating child 1" << std::endl;
    GenerateCBSChildNode(child[1], curr);
    PrintNode(*child[1]);

    PushCBSNodes(cbs, child[0], child[1]);
    curr->clear();
  }
  return total_profit;
}

void TeamOrienteeringProblemSolver::PrintNode(const Node& node)
{
  // std::cout << "(id: " << node.id_ << " t: " << node.t_ << " g: " << node.g_ << " f: " << node.f_
  //           << " vi: " << GetVisitedNodesString(node.visited_nodes_) << ")" << std::endl;
  std::cout << "(" << node.id_ << "-" << node.t_ << " g: " << node.g_ << " f: " << node.f_
            << " vi: " << GetVisitedNodesString(node.visited_nodes_) << " parent: " << node.parent_ << ")" << std::endl;
}

void TeamOrienteeringProblemSolver::PrintNode(const JointStateNode& node)
{
  for (int i = 0; i < node.nodes_.size(); i++)
  {
    std::cout << "n" << i << ": (" << node.nodes_[i].id_ << "-" << node.nodes_[i].t_ << " g: " << node.nodes_[i].g_
              << " f: " << node.nodes_[i].f_ << " vi: " << GetVisitedNodesString(node.nodes_[i].visited_nodes_) << ")"
              << std::endl;
  }
  std::cout << "overall f: " << node.f_ << " g: " << node.g_ << " t: " << node.t_ << " l: " << node.label_id_
            << " p: " << node.parent_ << std::endl;
}

void TeamOrienteeringProblemSolver::PrintNode(const PBS_ns::PBSNode& node)
{
  std::cout << "Node depth: " << node.depth << " profit: " << node.profit_ << std::endl;
  std::cout << "Paths: " << std::endl;
  for (const auto& path : node.paths_)
  {
    std::cout << path.first << ": ";
    for (const auto& node_id : path.second)
    {
      std::cout << node_id << " ";
    }
    std::cout << std::endl;
  }
  // for (int i = 0; i < vehicle_num_; i++)
  // {
  //   std::cout << i << ": ";
  //   for (int j = 0; j < node.paths_[i].size(); j++)
  //   {
  //     std::cout << node.paths_[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }
  std::cout << "Conflicts: " << std::endl;
  for (const auto& conflict : node.conflicts_)
  {
    std::cout << (*conflict).a1 << " " << (*conflict).a2 << std::endl;
  }
}

void TeamOrienteeringProblemSolver::PrintNode(const CBS_ns::CBSNode& node)
{
  std::cout << "Node depth: " << node.depth << " profit: " << node.profit_ << std::endl;
  std::cout << "Paths: " << std::endl;
  for (int i = 0; i < node.paths_.size(); i++)
  {
    std::cout << "v" << i << ": ";
    for (const auto& node_id : node.paths_[i])
    {
      std::cout << node_id << " ";
    }
    std::cout << std::endl;
  }
  // for (int i = 0; i < vehicle_num_; i++)
  // {
  //   std::cout << i << ": ";
  //   for (int j = 0; j < node.paths_[i].size(); j++)
  //   {
  //     std::cout << node.paths_[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }
  std::cout << "Conflicts: " << std::endl;
  for (const auto& conflict : node.conflicts_)
  {
    std::cout << "<" << (*conflict).a1 << " " << (*conflict).a2 << "-" << (*conflict).node_id << ">" << std::endl;
  }
}

// std::string TeamOrienteeringProblemSolver::GetVisitedNodesString(int visited_nodes)
std::string TeamOrienteeringProblemSolver::GetVisitedNodesString(IndexSet visited_nodes)
{
  std::string visited_nodes_str = "";
  for (int i = 0; i < graph_node_num_; i++)
  {
    // if (visited_nodes & (1 << i))
    if (visited_nodes.IsElementInSet(i))
    {
      visited_nodes_str += std::to_string(i) + " ";
    }
  }
  return visited_nodes_str;
}
