/**
 * @file PBS_utils.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Utility functions for PBS
 * @version 0.1
 * @date 2023-07-14
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

namespace PBS_ns
{
struct Constraint
{
  int low = -1;
  int high = -1;
  void set(int _low, int _high)
  {
    low = _low;
    high = _high;
  }
};

std::ostream& operator<<(std::ostream& os, const Constraint& constraint);

struct Conflict
{
  int a1;
  int a2;
  explicit Conflict(int a1 = -1, int a2 = -1) : a1(a1), a2(a2)
  {
  }
};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator<(const Conflict& conflict1, const Conflict& conflict2);

struct PathEntry
{
  int location = -1;
  explicit PathEntry(int loc = -1) : location(loc)
  {
  }
};

typedef std::vector<PathEntry> Path;
std::ostream& operator<<(std::ostream& os, const Path& path);

class PBSNode
{
public:
  Constraint constraint_;
  // std::list<std::pair<int, Path>> paths_;
  std::list<std::pair<int, std::vector<int>>> paths_;
  // conflicts in the current paths_
  std::list<std::shared_ptr<Conflict>> conflicts_;
  // The chosen conflict
  std::shared_ptr<Conflict> conflict_;

  double profit_;
  std::vector<double> profits_;

  size_t depth = 0;  // depath of this CT node

  uint64_t time_expanded = 0;
  uint64_t time_generated = 0;

  PBSNode* parent = nullptr;
  PBSNode* children[2] = { nullptr, nullptr };

  PBSNode() : profit_(0.0)
  {
  }

  PBSNode(PBSNode& parent)
    : profit_(parent.profit_)
    , profits_(parent.profits_)
    , depth(parent.depth + 1)
    , conflicts_(parent.conflicts_)
    , parent(&parent)
  {
  }
  ~PBSNode() = default;

  void clear();
  void printConstraints(int id) const;
  inline int getNumNewPaths() const
  {
    return (int)paths_.size();
  }
};

std::ostream& operator<<(std::ostream& os, const PBSNode& node);

class PBS
{
public:
  bool solution_found = false;
  double solution_profit;

  // std::vector<Path*> paths_;
  std::vector<std::vector<int>> paths_;
  std::vector<std::vector<bool>> priority_graph_;  // [i][j] = true indicates that i is lower than j
  std::list<int> ordered_agents_;

  PBS(int num_vehicles);
  ~PBS();
  // bool solve(int runtime_limit, std::function<void(int, int, int, std::vector<int>&)> single_vehicle_solver,
  //            int start_node_id, int end_goal_id, int time_budget, std::vector<int>& path);

  bool terminate(PBSNode* curr);  // check the stop condition and return true if it meets
  inline bool open_list_empty()
  {
    return open_list_.empty();
  }
  PBSNode* selectNode();

  void pushNode(PBSNode* node);
  void pushNodes(PBSNode* n1, PBSNode* n2);
  std::shared_ptr<Conflict> chooseConflict(const PBSNode& node) const;
  bool hasHigherPriority(int low, int high) const;  // return true if agent low is lower than agent high
  bool hasConflicts(int a1, int a2) const;

  void topologicalSort(std::list<int>& stack);
  void getHigherPriorityAgents(const std::list<int>::reverse_iterator& p1, std::set<int>& higher_agents);
  void getLowerPriorityAgents(const std::list<int>::iterator& p1, std::set<int>& lower_subplans);

private:
  int num_vehicles_;
  std::stack<PBSNode*> open_list_;
  std::list<PBSNode*> allNodes_table_;

  inline void releaseNodes();

  // bool generateRoot();
  // bool generateChild(int child_id, PBSNode* parent, int low, int high);

  bool findPathForSingleAgent(PBSNode& node, const std::set<int>& higher_agents, int a, Path& new_path);
  void classifyConflicts(PBSNode& parent);
  void update(PBSNode* node);

  // Utility functions
  void topologicalSortUtil(int v, std::vector<bool>& visited, std::list<int>& stack);

  // Printing for debugging
  void printPaths() const;
};
}  // namespace PBS_ns
