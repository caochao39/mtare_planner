/**
 * @file CBS_utils.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Utility functions for CBS
 * @version 0.1
 * @date 2023-07-31
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <list>
#include <memory>
#include <utils/misc_utils.h>
#include <functional>

namespace CBS_ns
{
///// Constraint
typedef std::tuple<int, int> Constraint;  // vehicle_id is not allowed to visit node_id

///// Conflict
struct Conflict
{
  int a1;
  int a2;
  int node_id;
  Constraint constraint1;
  Constraint constraint2;

  explicit Conflict(int a1, int a2, int node_id) : a1(a1), a2(a2), node_id(node_id)
  {
    constraint1 = std::make_tuple(a1, node_id);
    constraint2 = std::make_tuple(a2, node_id);
  }
};

std::ostream& operator<<(std::ostream& os, const Constraint& constraint);
std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator<(const Conflict& conflict1, const Conflict& conflict2);

///// Path (tmp)
struct PathEntry
{
  int location = -1;
  explicit PathEntry(int loc = -1) : location(loc)
  {
  }
};

typedef std::vector<PathEntry> Path;
std::ostream& operator<<(std::ostream& os, const Path& path);

///// CBSNode
class CBSNode
{
public:
  double g_;
  double h_;
  double cost_to_go_;  // informed but admissible h
  size_t depth_ = 0;
  bool h_computed_ = false;
  double profit_;
  std::vector<double> profits_;

  std::list<std::shared_ptr<Conflict>> conflicts_;
  std::shared_ptr<Conflict> conflict_;  // The chosen conflict

  std::list<Constraint> constraints_;

  std::vector<std::vector<int>> paths_;

  CBSNode* parent = nullptr;

  size_t depth = 0;  // depth of this CT node

  uint64_t time_expanded = 0;
  uint64_t time_generated = 0;

  CBSNode();

  CBSNode(CBSNode& parent);

  ~CBSNode() = default;

  inline double getF() const
  {
    return g_ + h_;
  }
  void clear();
};

std::ostream& operator<<(std::ostream& os, const CBSNode& node);

struct CompareCBSNodeByF
{
  bool operator()(const CBSNode* n1, const CBSNode* n2) const
  {
    if (n1->g_ + n1->h_ == n2->g_ + n2->h_)
    {
      return n1->h_ <= n2->h_;
    }
    return n1->g_ + n1->h_ <= n2->g_ + n2->h_;
  }
};

class CBS
{
public:
  bool solution_found = false;
  double solution_profit;

  // std::vector<Path*> paths_;
  std::vector<std::vector<int>> paths_;
  std::vector<std::vector<bool>> priority_graph_;  // [i][j] = true indicates that i is lower than j
  std::list<int> ordered_agents_;

  CBS(int num_vehicles);
  ~CBS();
  bool solve(int runtime_limit, std::function<void(int, int, int, std::vector<int>&)> single_vehicle_solver,
             int start_node_id, int end_goal_id, int time_budget, std::vector<int>& path);

  bool terminate(CBSNode* curr);  // check the stop condition and return true if it meets
  inline bool open_list_empty()
  {
    return open_list_.empty();
  }
  CBSNode* selectNode();

  void pushNode(CBSNode* node);
  void pushNodes(CBSNode* n1, CBSNode* n2);
  std::shared_ptr<Conflict> chooseConflict(const CBSNode& node) const;
  // bool hasHigherPriority(int low, int high) const;  // return true if agent low is lower than agent high

  // bool hasConflicts(int a1, int a2) const;

  // void topologicalSort(std::list<int>& stack);
  // void getHigherPriorityAgents(const std::list<int>::reverse_iterator& p1, std::set<int>& higher_agents);
  // void getLowerPriorityAgents(const std::list<int>::iterator& p1, std::set<int>& lower_subplans);

private:
  int num_vehicles_;
  std::list<CBSNode*> allNodes_table_;

  std::priority_queue<CBSNode*, std::vector<CBSNode*>, CompareCBSNodeByF> open_list_;

  inline void releaseNodes();

  // bool generateRoot();
  // bool generateChild(int child_id, CBSNode* parent, int low, int high);

  // bool findPathForSingleAgent(CBSNode& node, const std::set<int>& higher_agents, int a, Path& new_path);
  void classifyConflicts(CBSNode& parent);
  // void update(CBSNode* node);

  // Utility functions
  void topologicalSortUtil(int v, std::vector<bool>& visited, std::list<int>& stack);

  // Printing for debugging
  void printPaths() const;
};
}  // namespace CBS_ns
