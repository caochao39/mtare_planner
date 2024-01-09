/**
 * @file graph.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a graph
 * @version 0.1
 * @date 2021-07-11
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>

#include <Eigen/Core>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <utils/misc_utils.h>

namespace tare
{
template <class T>
class Graph
{
public:
  explicit Graph();
  ~Graph() = default;

  bool HasNode(int node_id) const
  {
    return node_id_to_index_.find(node_id) != node_id_to_index_.end();
  }
  T& GetNodeByIndex(int node_index);
  T& GetNodeByID(int node_id);
  void AddNode(const T& node);
  void AddEdge(int from_node_id, int to_node_id);
  bool IsConnected(int from_node_id, int to_node_id);
  bool IsCutOff(int from_node_id, int to_node_id);
  void CutOffEdge(int from_node_id, int to_node_id);
  double GetEdgeDistance(int from_node_id, int to_node_id);
  void SetEdgeDistance(int from_node_id, int to_node_id, double distance);
  void ResetEdgeDistance(int from_node_id, int to_node_id);
  void RemoveEdge(int from_node_id, int to_node_id);
  void GetNeighborDistances(int node_id, std::vector<double>& neighbor_distances);
  int GetNodeNumber() const;
  void GetNodeIDs(std::vector<int>& node_ids) const;
  int GetNodeDegrees(int node_id);
  double GetShortestPath(int from_node_id, int to_node_id, bool get_path, nav_msgs::Path& shortest_path,
                         std::vector<int>& node_ids, bool ignore_cutoff_edge = true);
  void GetVisualizationMarkers(visualization_msgs::Marker& node_marker, visualization_msgs::Marker& edge_marker) const;
  void GetEdgeWeightMarkers(visualization_msgs::MarkerArray& edge_weight_marker) const;

  void SetDistanceFunction(double (*distance_function)(const T& node1, const T& node2))
  {
    distance_function_ = distance_function;
  }

  void SetEdgeCutOffDistanceThreshold(double distance_cut_off_threshold)
  {
    kEdgeCutOffDistanceThreshold = distance_cut_off_threshold;
  }

  double GetEdgeCutOffDistanceThreshold()
  {
    return kEdgeCutOffDistanceThreshold;
  }

  void SetPositionFunction(geometry_msgs::Point (*position_function)(const T& node))
  {
    position_function_ = position_function;
  }

  void SetIDFunction(int (*id_function)(const T& node))
  {
    id_function_ = id_function;
  }

  void SetNodeConstraintFunction(bool (*node_constraint_function)(const T& node))
  {
    node_constraint_function_ = node_constraint_function;
  }

  void SetNodes(const std::vector<T>& nodes)
  {
    nodes_ = nodes;
    UpdateIDMap();
  }

  void SetConnectionMatrix(const std::vector<std::vector<int>>& connection_matrix)
  {
    connection_ = connection_matrix;
  }

  void SetDistanceMatrix(const std::vector<std::vector<double>>& distance_matrix)
  {
    distance_ = distance_matrix;
  }

  void UpdateIDMap()
  {
    for (int i = 0; i < nodes_.size(); i++)
    {
      int node_id = id_function_(nodes_[i]);
      node_id_to_index_[node_id] = i;
    }
  }

  double GetDistanceBetweenNodes(const T& node1, const T& node2) const
  {
    MY_ASSERT(distance_function_ != nullptr);
    return distance_function_(node1, node2);
  }

  geometry_msgs::Point GetNodePosition(const T& node) const
  {
    MY_ASSERT(position_function_ != nullptr);
    return position_function_(node);
  }

  int GetNodeID(const T& node) const
  {
    MY_ASSERT(id_function_ != nullptr);
    return id_function_(node);
  }

  int GetNodeID(int node_index) const
  {
    MY_ASSERT(node_index >= 0 && node_index < nodes_.size());
    return GetNodeID(nodes_[node_index]);
  }

  // Returns pairs of node_ids
  void GetEdges(std::vector<std::pair<int, int>>& edges) const;

  // Save connection and distance matrices to file
  void SaveToFile(const std::string& node_filename, const std::string& connection_matrix_filename,
                  const std::string& distance_matrix_filename) const;
  void ReadFromFile(const std::string& node_filename, const std::string& connection_matrix_filename,
                    const std::string& distance_matrix_filename);
  void ClearNodes();

private:
  bool NodeIndexInRange(int node_index) const
  {
    return node_index >= 0 && node_index < connection_.size();
  }
  double AStarSearch(int from_node_index, int to_node_index, bool get_path, std::vector<int>& node_indices,
                     bool ignore_cutoff_edge = true) const;
  // Node connectivity
  std::vector<std::vector<int>> connection_;
  // Distances between two nodes
  std::vector<std::vector<double>> distance_;
  // Nodes
  std::vector<T> nodes_;
  // Node id to node list index mapping
  std::unordered_map<int, int> node_id_to_index_;
  // Distance function for path searching
  double (*distance_function_)(const T& node1, const T& node2);
  // Position function for getting the position of a node
  geometry_msgs::Point (*position_function_)(const T& node);
  // id function for getting the id of a node
  int (*id_function_)(const T& node);
  // Constraint function for a node to be considered feasible on a path
  bool (*node_constraint_function_)(const T& node);
  // Distance threshold to not consider an edge when searching for a path
  double kEdgeCutOffDistanceThreshold;
};

template <class T>
Graph<T>::Graph()
  : distance_function_(nullptr)
  , id_function_(nullptr)
  , position_function_(nullptr)
  , node_constraint_function_([](const T& node) { return true; })
  , kEdgeCutOffDistanceThreshold(misc_utils_ns::INF_DISTANCE - 1)
{
  for (int i = 0; i < distance_.size(); i++)
  {
    for (int j = 0; j < distance_.size(); j++)
    {
      if (i == j)
      {
        distance_[i][j] = 0.0;
      }
      else
      {
        distance_[i][j] = DBL_MAX;
      }
    }
  }
}

template <class T>
T& Graph<T>::GetNodeByIndex(int node_index)
{
  MY_ASSERT(node_index >= 0 && node_index < nodes_.size());
  return nodes_[node_index];
}

template <class T>
T& Graph<T>::GetNodeByID(int node_id)
{
  MY_ASSERT(HasNode(node_id));
  return nodes_[node_id_to_index_[node_id]];
}

template <class T>
void Graph<T>::AddNode(const T& node)
{
  MY_ASSERT(id_function_ != nullptr);
  int node_id = id_function_(node);
  if (HasNode(node_id))
  {
    // Node exists
    nodes_[node_id_to_index_[node_id]] = node;
  }
  else
  {
    // Add a new node
    node_id_to_index_[node_id] = nodes_.size();
    nodes_.push_back(node);
    std::vector<int> connection;
    connection_.push_back(connection);
    std::vector<double> neighbor_distance;
    distance_.push_back(neighbor_distance);
  }
}

template <class T>
void Graph<T>::AddEdge(int from_node_id, int to_node_id)
{
  MY_ASSERT(HasNode(from_node_id) && HasNode(to_node_id));

  int from_node_index = node_id_to_index_[from_node_id];
  int to_node_index = node_id_to_index_[to_node_id];
  MY_ASSERT(NodeIndexInRange(from_node_index) && NodeIndexInRange(to_node_index));
  if (misc_utils_ns::ElementExistsInVector<int>(connection_[from_node_index], to_node_index) ||
      misc_utils_ns::ElementExistsInVector<int>(connection_[to_node_index], from_node_index))
  {
    return;
  }

  MY_ASSERT(distance_function_ != nullptr);
  double distance = distance_function_(nodes_[from_node_index], nodes_[to_node_index]);

  connection_[from_node_index].push_back(to_node_index);
  connection_[to_node_index].push_back(from_node_index);

  distance_[from_node_index].push_back(distance);
  distance_[to_node_index].push_back(distance);
}

template <class T>
bool Graph<T>::IsConnected(int from_node_id, int to_node_id)
{
  if (!HasNode(from_node_id) || !HasNode(to_node_id))
  {
    return false;
  }
  int from_node_index = node_id_to_index_[from_node_id];
  int to_node_index = node_id_to_index_[to_node_id];
  MY_ASSERT(NodeIndexInRange(from_node_index) && NodeIndexInRange(to_node_index));
  if (misc_utils_ns::ElementExistsInVector<int>(connection_[from_node_index], to_node_index) &&
      misc_utils_ns::ElementExistsInVector<int>(connection_[to_node_index], from_node_index))
  {
    return true;
  }
  else
  {
    return false;
  }
}

template <class T>
bool Graph<T>::IsCutOff(int from_node_id, int to_node_id)
{
  if (IsConnected(from_node_id, to_node_id))
  {
    if (GetEdgeDistance(from_node_id, to_node_id) > kEdgeCutOffDistanceThreshold)
    {
      return true;
    }
  }
  return false;
}

template <class T>
void Graph<T>::CutOffEdge(int from_node_id, int to_node_id)
{
  SetEdgeDistance(from_node_id, to_node_id, misc_utils_ns::INF_DISTANCE);
}

template <class T>
double Graph<T>::GetEdgeDistance(int from_node_id, int to_node_id)
{
  if (HasNode(from_node_id) && HasNode(to_node_id) && from_node_id == to_node_id)
  {
    return 0.0;
  }

  if (IsConnected(from_node_id, to_node_id))
  {
    int from_node_index = node_id_to_index_[from_node_id];
    int to_node_index = node_id_to_index_[to_node_id];
    for (int i = 0; i < connection_[from_node_index].size(); i++)
    {
      if (connection_[from_node_index][i] == to_node_index)
      {
        return distance_[from_node_index][i];
      }
    }
  }
  return misc_utils_ns::INF_DISTANCE;
}

template <class T>
void Graph<T>::SetEdgeDistance(int from_node_id, int to_node_id, double distance)
{
  if (IsConnected(from_node_id, to_node_id))
  {
    int from_node_index = node_id_to_index_[from_node_id];
    int to_node_index = node_id_to_index_[to_node_id];
    for (int i = 0; i < connection_[from_node_index].size(); i++)
    {
      if (connection_[from_node_index][i] == to_node_index)
      {
        distance_[from_node_index][i] = distance;
        break;
      }
    }
    for (int i = 0; i < connection_[to_node_index].size(); i++)
    {
      if (connection_[to_node_index][i] == from_node_index)
      {
        distance_[to_node_index][i] = distance;
        break;
      }
    }
  }
}

template <class T>
void Graph<T>::ResetEdgeDistance(int from_node_id, int to_node_id)
{
  if (IsConnected(from_node_id, to_node_id))
  {
    int from_node_index = node_id_to_index_[from_node_id];
    int to_node_index = node_id_to_index_[to_node_id];
    double distance = distance_function_(nodes_[from_node_index], nodes_[to_node_index]);
    for (int i = 0; i < connection_[from_node_index].size(); i++)
    {
      if (connection_[from_node_index][i] == to_node_index)
      {
        distance_[from_node_index][i] = distance;
        break;
      }
    }
    for (int i = 0; i < connection_[to_node_index].size(); i++)
    {
      if (connection_[to_node_index][i] == from_node_index)
      {
        distance_[to_node_index][i] = distance;
        break;
      }
    }
  }
}

template <class T>
void Graph<T>::RemoveEdge(int from_node_id, int to_node_id)
{
  MY_ASSERT(HasNode(from_node_id) && HasNode(to_node_id));

  int from_node_index = node_id_to_index_[from_node_id];
  int to_node_index = node_id_to_index_[to_node_id];
  MY_ASSERT(NodeIndexInRange(from_node_index) && NodeIndexInRange(to_node_index));

  for (int i = 0; i < connection_[from_node_index].size(); i++)
  {
    if (connection_[from_node_index][i] == to_node_index)
    {
      connection_[from_node_index].erase(connection_[from_node_index].begin() + i);
      distance_[from_node_index].erase(distance_[from_node_index].begin() + i);
      i--;
    }
  }

  for (int i = 0; i < connection_[to_node_index].size(); i++)
  {
    if (connection_[to_node_index][i] == from_node_index)
    {
      connection_[to_node_index].erase(connection_[to_node_index].begin() + i);
      distance_[to_node_index].erase(distance_[to_node_index].begin() + i);
      i--;
    }
  }
}

template <class T>
void Graph<T>::GetNeighborDistances(int node_id, std::vector<double>& neighbor_distances)
{
  neighbor_distances.clear();
  if (!HasNode(node_id))
  {
    return;
  }
  int node_index = node_id_to_index_[node_id];
  MY_ASSERT(node_index >= 0 && node_index < distance_.size());
  for (int i = 0; i < distance_[node_index].size(); i++)
  {
    neighbor_distances.push_back(distance_[node_index][i]);
  }
}

template <class T>
void Graph<T>::GetEdges(std::vector<std::pair<int, int>>& edges) const
{
  edges.clear();
  for (int i = 0; i < connection_.size(); i++)
  {
    int start_ind = i;
    for (int j = 0; j < connection_[i].size(); j++)
    {
      int end_ind = connection_[i][j];

      int from_node_id = id_function_(nodes_[start_ind]);
      int to_node_id = id_function_(nodes_[end_ind]);

      if (from_node_id > to_node_id)
      {
        std::swap(from_node_id, to_node_id);
      }
      if (std::find(edges.begin(), edges.end(), std::make_pair(from_node_id, to_node_id)) == edges.end())
      {
        edges.emplace_back(from_node_id, to_node_id);
      }
    }
  }
}

template <class T>
int Graph<T>::GetNodeNumber() const
{
  return nodes_.size();
}

template <class T>
void Graph<T>::GetNodeIDs(std::vector<int>& node_ids) const
{
  node_ids.clear();
  for (const auto node : nodes_)
  {
    node_ids.push_back(id_function_(node));
  }
}

template <class T>
int Graph<T>::GetNodeDegrees(int node_id)
{
  MY_ASSERT(HasNode(node_id));

  int node_index = node_id_to_index_[node_id];
  return connection_[node_index].size();
}

template <class T>
double Graph<T>::GetShortestPath(int from_node_id, int to_node_id, bool get_path, nav_msgs::Path& shortest_path,
                                 std::vector<int>& node_ids, bool ignore_cutoff_edge)
{
  MY_ASSERT(distance_function_ != nullptr);
  MY_ASSERT(id_function_ != nullptr);
  MY_ASSERT(HasNode(from_node_id));
  MY_ASSERT(HasNode(to_node_id));

  int from_node_index = node_id_to_index_[from_node_id];
  int to_node_index = node_id_to_index_[to_node_id];

  std::vector<int> node_indices;
  double path_length = AStarSearch(from_node_index, to_node_index, get_path, node_indices, ignore_cutoff_edge);

  shortest_path.poses.clear();
  node_ids.clear();
  if (get_path && !node_indices.empty())
  {
    for (const auto& node_index : node_indices)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position = position_function_(nodes_[node_index]);
      shortest_path.poses.push_back(pose);
    }

    for (const auto& node_index : node_indices)
    {
      node_ids.push_back(id_function_(nodes_[node_index]));
    }
  }
  return path_length;
}

template <class T>
void Graph<T>::GetVisualizationMarkers(visualization_msgs::Marker& node_marker,
                                       visualization_msgs::Marker& edge_marker) const
{
  node_marker.points.clear();
  edge_marker.points.clear();
  edge_marker.colors.clear();

  std_msgs::ColorRGBA EdgeColor;
  EdgeColor.r = 0.0;
  EdgeColor.g = 0.0;
  EdgeColor.b = 1.0;
  EdgeColor.a = 0.8;

  std_msgs::ColorRGBA CutOffEdgeColor;
  CutOffEdgeColor.r = 1.0;
  CutOffEdgeColor.g = 0.0;
  CutOffEdgeColor.b = 0.0;
  CutOffEdgeColor.a = 0.8;

  for (const auto& node : nodes_)
  {
    node_marker.points.push_back(position_function_(node));
  }

  std::vector<std::pair<int, int>> added_edge;
  for (int i = 0; i < connection_.size(); i++)
  {
    int start_ind = i;
    for (int j = 0; j < connection_[i].size(); j++)
    {
      int end_ind = connection_[i][j];
      if (std::find(added_edge.begin(), added_edge.end(), std::make_pair(start_ind, end_ind)) == added_edge.end())
      {
        geometry_msgs::Point start_node_position = position_function_(nodes_[start_ind]);
        geometry_msgs::Point end_node_position = position_function_(nodes_[end_ind]);

        edge_marker.points.push_back(start_node_position);
        edge_marker.points.push_back(end_node_position);
        if (distance_[i][j] > kEdgeCutOffDistanceThreshold)
        {
          edge_marker.colors.push_back(CutOffEdgeColor);
          edge_marker.colors.push_back(CutOffEdgeColor);
        }
        else
        {
          edge_marker.colors.push_back(EdgeColor);
          edge_marker.colors.push_back(EdgeColor);
        }
        added_edge.emplace_back(start_ind, end_ind);
        added_edge.emplace_back(end_ind, start_ind);
      }
    }
  }
}

template <class T>
void Graph<T>::GetEdgeWeightMarkers(visualization_msgs::MarkerArray& edge_weight_marker) const
{
  edge_weight_marker.markers.clear();

  // Add node label
  for (int i = 0; i < nodes_.size(); i++)
  {
    geometry_msgs::Point node_position = position_function_(nodes_[i]);
    visualization_msgs::Marker text_marker;
    text_marker.header.stamp = ros::Time::now();
    text_marker.header.frame_id = "map";
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.id = edge_weight_marker.markers.size();
    text_marker.scale.z = 0.5;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.text = std::to_string(i);
    node_position.x += 1;
    node_position.y += 1;
    text_marker.pose.position = node_position;
    edge_weight_marker.markers.push_back(text_marker);
  }

  std::vector<std::pair<int, int>> added_edge;
  for (int i = 0; i < connection_.size(); i++)
  {
    int start_ind = i;
    for (int j = 0; j < connection_[i].size(); j++)
    {
      int end_ind = connection_[i][j];
      if (std::find(added_edge.begin(), added_edge.end(), std::make_pair(start_ind, end_ind)) == added_edge.end())
      {
        geometry_msgs::Point start_node_position = position_function_(nodes_[start_ind]);
        geometry_msgs::Point end_node_position = position_function_(nodes_[end_ind]);
        geometry_msgs::Point edge_mid_position;
        edge_mid_position.x = (start_node_position.x + end_node_position.x) / 2;
        edge_mid_position.y = (start_node_position.y + end_node_position.y) / 2;
        edge_mid_position.z = (start_node_position.z + end_node_position.z) / 2;

        visualization_msgs::Marker text_marker;
        text_marker.header.stamp = ros::Time::now();
        text_marker.header.frame_id = "map";
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.id = edge_weight_marker.markers.size();
        text_marker.scale.z = 0.1;
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.text = std::to_string(distance_[i][j]);
        text_marker.pose.position = edge_mid_position;
        edge_weight_marker.markers.push_back(text_marker);

        added_edge.emplace_back(start_ind, end_ind);
        added_edge.emplace_back(end_ind, start_ind);
      }
    }
  }
}

template <class T>
double Graph<T>::AStarSearch(int from_node_index, int to_node_index, bool get_path, std::vector<int>& node_indices,
                             bool ignore_cutoff_edge) const
{
  MY_ASSERT(NodeIndexInRange(from_node_index));
  MY_ASSERT(NodeIndexInRange(to_node_index));
  MY_ASSERT(distance_function_ != nullptr);

  typedef std::pair<double, int> iPair;
  double shortest_dist = misc_utils_ns::INF_DISTANCE;
  std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> frontiers;
  std::vector<double> cost_from_start(connection_.size(), misc_utils_ns::INF_DISTANCE);
  std::vector<double> cost(connection_.size(), misc_utils_ns::INF_DISTANCE);
  std::vector<double> heuristic(connection_.size(), misc_utils_ns::INF_DISTANCE);
  std::vector<bool> h_computed(connection_.size(), false);
  std::vector<int> prev(connection_.size(), -1);
  std::vector<bool> in_pg(connection_.size(), false);

  cost_from_start[from_node_index] = 0;
  heuristic[from_node_index] = distance_function_(nodes_[from_node_index], nodes_[to_node_index]);
  h_computed[from_node_index] = true;
  cost[from_node_index] = cost_from_start[from_node_index] + heuristic[from_node_index];

  frontiers.push(std::make_pair(heuristic[from_node_index], from_node_index));

  // An alternative implementation without the need to perform decrease-key operations
  bool found_path = false;
  while (!frontiers.empty())
  {
    int curr_node_index = frontiers.top().second;
    double curr_node_cost = frontiers.top().first;
    frontiers.pop();
    if (curr_node_index == to_node_index)
    {
      shortest_dist = cost_from_start[curr_node_index];
      found_path = true;
      break;
    }
    if (curr_node_cost <= cost[curr_node_index])
    {
      for (int i = 0; i < connection_[curr_node_index].size(); i++)
      {
        int neighbor_node_index = connection_[curr_node_index][i];
        MY_ASSERT(misc_utils_ns::InRange<std::vector<int>>(connection_, neighbor_node_index));
        if (!node_constraint_function_(nodes_[neighbor_node_index]))
        {
          continue;
        }
        double edge_distance = distance_[curr_node_index][i];
        if (!ignore_cutoff_edge && edge_distance > kEdgeCutOffDistanceThreshold)
        {
          continue;
        }
        if (cost_from_start[neighbor_node_index] > cost_from_start[curr_node_index] + edge_distance)
        {
          prev[neighbor_node_index] = curr_node_index;
          cost_from_start[neighbor_node_index] = cost_from_start[curr_node_index] + edge_distance;
          if (!h_computed[neighbor_node_index])
          {
            heuristic[neighbor_node_index] = distance_function_(nodes_[neighbor_node_index], nodes_[to_node_index]);
            h_computed[neighbor_node_index] = true;
          }
          cost[neighbor_node_index] = cost_from_start[neighbor_node_index] + heuristic[neighbor_node_index];
          frontiers.push(std::make_pair(cost[neighbor_node_index], neighbor_node_index));
        }
      }
    }
  }

  node_indices.clear();
  if (found_path && get_path)
  {
    int curr_node_index = to_node_index;
    if (prev[curr_node_index] != -1 || curr_node_index == from_node_index)
    {
      while (curr_node_index != -1)
      {
        node_indices.push_back(curr_node_index);
        curr_node_index = prev[curr_node_index];
      }
    }
    std::reverse(node_indices.begin(), node_indices.end());
  }

  return found_path ? shortest_dist : misc_utils_ns::INF_DISTANCE;
}

template <class T>
void Graph<T>::SaveToFile(const std::string& node_filename, const std::string& connection_matrix_filename,
                          const std::string& distance_matrix_filename) const
{
  misc_utils_ns::SaveVectorToFile<T>(nodes_, node_filename);
  misc_utils_ns::Save2DVectorToFile<int>(connection_, connection_matrix_filename);
  misc_utils_ns::Save2DVectorToFile<double>(distance_, distance_matrix_filename);
}

template <class T>
void Graph<T>::ReadFromFile(const std::string& node_filename, const std::string& connection_matrix_filename,
                            const std::string& distance_matrix_filename)
{
  nodes_.clear();
  misc_utils_ns::ReadVectorFromFile<T>(nodes_, node_filename);

  UpdateIDMap();

  connection_.clear();
  misc_utils_ns::Read2DVectorFromFile<int>(connection_, connection_matrix_filename);

  distance_.clear();
  misc_utils_ns::Read2DVectorFromFile<double>(distance_, distance_matrix_filename);
}

template <class T>
void Graph<T>::ClearNodes()
{
  nodes_.clear();
  connection_.clear();
  distance_.clear();
  node_id_to_index_.clear();
}

}  // namespace tare
