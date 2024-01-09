/**
 * @file PBS_utils.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Utility functions for PBS
 * @version 0.1
 * @date 2023-07-14
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <utils/PBS_utils.h>
namespace PBS_ns
{
std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
{
  os << "<" << constraint.low << " is lower than " << constraint.high << ">";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
{
  os << "<" << conflict.a1 << "," << conflict.a2 << ">";
  return os;
}

bool operator<(const Conflict& conflict1, const Conflict& conflict2)  // return true if conflict2 has higher priority
{
  return rand() % 2;
}

void PBSNode::clear()
{
  conflicts_.clear();
}

void PBSNode::printConstraints(int id) const
{
  auto curr = this;
  while (curr->parent != nullptr)
  {
    std::cout << curr->constraint_.high << ">" << curr->constraint_.low << std::endl;
    curr = curr->parent;
  }
}

std::ostream& operator<<(std::ostream& os, const PBSNode& node)
{
  os << "Node " << node.time_generated << " (profit=" << node.profit_ << ", conflicts=" << node.conflicts_.size()
     << ") with " << node.getNumNewPaths() << " new paths";
  return os;
}

PBS::PBS(int num_vehicles) : num_vehicles_(num_vehicles)
{
}

PBS::~PBS()
{
  releaseNodes();
}

inline void PBS::releaseNodes()
{
  // TODO:: clear open_list_
  for (auto& node : allNodes_table_)
    delete node;
  allNodes_table_.clear();
}

// bool PBS::solve(int runtime_limit)
// {
//   generateRoot();

//   while (!open_list_.empty())
//   {
//     auto curr = selectNode();

//     if (terminate(curr))
//       break;

//     curr->conflict = chooseConflict(*curr);

//     // if (screen > 1)
//     //   cout << "	Expand " << *curr << "	on " << *(curr->conflict) << endl;

//     MY_ASSERT(!hasHigherPriority(curr->conflict->a1, curr->conflict->a2) and
//               !hasHigherPriority(curr->conflict->a2, curr->conflict->a1));
//     auto t1 = clock();
//     std::vector<Path*> copy(paths_);
//     generateChild(0, curr, curr->conflict->a1, curr->conflict->a2);
//     paths_ = copy;
//     generateChild(1, curr, curr->conflict->a2, curr->conflict->a1);
//     pushNodes(curr->children[0], curr->children[1]);
//     curr->clear();
//   }  // end of while loop
//   return solution_found;
// }

bool PBS::terminate(PBSNode* curr)
{
  if (curr->conflicts_.empty())
  {
    return true;
  }

  // TODO: check runtime limit
  return false;
}

void PBS::pushNode(PBSNode* node)
{
  open_list_.push(node);
  allNodes_table_.push_back(node);
}

void PBS::pushNodes(PBSNode* n1, PBSNode* n2)
{
}

PBSNode* PBS::selectNode()
{
  PBSNode* curr = open_list_.top();
  open_list_.pop();
  update(curr);
  return curr;
}

// bool PBS::generateRoot()
// {
//   auto root = new PBSNode();
//   root->profit = 0.0;
//   paths_.reserve(num_of_vehicles);
// }
// bool PBS::generateChild(int child_id, PBSNode* parent, int low, int high)
// {
// }

bool PBS::hasConflicts(int a1, int a2) const
{
  std::vector<int> common;
  misc_utils_ns::GetCommonElements(paths_[a1], paths_[a2], common);

  return !common.empty();
}

std::shared_ptr<Conflict> PBS::chooseConflict(const PBSNode& node) const
{
  if (node.conflicts_.empty())
  {
    return nullptr;
  }
  return node.conflicts_.back();
}

void PBS::getHigherPriorityAgents(const std::list<int>::reverse_iterator& p1, std::set<int>& higher_agents)
{
  for (auto p2 = std::next(p1); p2 != ordered_agents_.rend(); ++p2)
  {
    if (priority_graph_[*p1][*p2])
    {
      auto ret = higher_agents.insert(*p2);
      if (ret.second)  // insert successfully
      {
        getHigherPriorityAgents(p2, higher_agents);
      }
    }
  }
}
void PBS::getLowerPriorityAgents(const std::list<int>::iterator& p1, std::set<int>& lower_subplans)
{
  for (auto p2 = std::next(p1); p2 != ordered_agents_.end(); ++p2)
  {
    if (priority_graph_[*p2][*p1])
    {
      auto ret = lower_subplans.insert(*p2);
      if (ret.second)  // insert successfully
      {
        getLowerPriorityAgents(p2, lower_subplans);
      }
    }
  }
}
bool PBS::hasHigherPriority(int low, int high) const
{
  std::queue<int> Q;
  std::vector<bool> visited(num_vehicles_, false);
  visited[low] = false;
  Q.push(low);
  while (!Q.empty())
  {
    auto n = Q.front();
    Q.pop();
    if (n == high)
    {
      return true;
    }
    for (int i = 0; i < num_vehicles_; i++)
    {
      if (priority_graph_[n][i] && !visited[i])
      {
        Q.push(i);
      }
    }
  }
  return false;
}  // return true if agent low is lower than agent high

bool PBS::findPathForSingleAgent(PBSNode& node, const std::set<int>& higher_agents, int a, Path& new_path)
{
  return false;
}
void PBS::classifyConflicts(PBSNode& parent)
{
}
void PBS::update(PBSNode* node)
{
  for (int i = 0; i < paths_.size(); i++)
  {
    paths_[i].clear();
  }
  priority_graph_.assign(num_vehicles_, std::vector<bool>(num_vehicles_, false));
  for (auto curr = node; curr != nullptr; curr = curr->parent)
  {
    for (const auto& path : curr->paths_)
    {
      if (paths_[path.first].empty())
      {
        paths_[path.first] = path.second;
      }
    }

    if (curr->parent != nullptr)  // non-root node
    {
      priority_graph_[curr->constraint_.low][curr->constraint_.high] = true;
    }
  }
}

void PBS::topologicalSort(std::list<int>& stack)
{
  stack.clear();
  std::vector<bool> visited(num_vehicles_, false);

  // Call the recursive helper function to store Topological
  // Sort starting from all vertices one by one
  for (int i = 0; i < num_vehicles_; i++)
  {
    if (!visited[i])
      topologicalSortUtil(i, visited, stack);
  }
}
void PBS::topologicalSortUtil(int v, std::vector<bool>& visited, std::list<int>& stack)
{
  // Mark the current node as visited.
  visited[v] = true;

  // Recur for all the vertices adjacent to this vertex
  MY_ASSERT(!priority_graph_.empty());
  for (int i = 0; i < num_vehicles_; i++)
  {
    if (priority_graph_[v][i] && !visited[i])
      topologicalSortUtil(i, visited, stack);
  }
  // Push current vertex to stack which stores result
  stack.push_back(v);
}

void PBS::printPaths() const
{
}
}  // namespace PBS_ns
