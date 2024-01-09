/**
 * @file directed_acyclic_graph.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a knowledge base
 * @version 0.1
 * @date 2023-04-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <utils/graph/directed_acyclic_graph.h>
#include <fstream>
#include <queue>
#include <utils/misc_utils.h>

std::vector<int> DirectedAcyclicGraph::topologicalSort()
{
  int numVertices = adj.size();
  std::vector<int> inDegree(numVertices, 0);

  for (const auto& neighbors : adj)
  {
    for (const auto& neighbor : neighbors)
    {
      inDegree[neighbor.first]++;
    }
  }

  std::queue<int> q;
  for (int i = 0; i < numVertices; i++)
  {
    if (inDegree[i] == 0)
    {
      q.push(i);
    }
  }

  std::vector<int> sorted;
  while (!q.empty())
  {
    int curr = q.front();
    q.pop();

    sorted.push_back(curr);

    for (const auto& neighbor : adj[curr])
    {
      inDegree[neighbor.first]--;
      if (inDegree[neighbor.first] == 0)
      {
        q.push(neighbor.first);
      }
    }
  }

  return sorted;
}

double DirectedAcyclicGraph::longestPath(int startNode, int endNode, std::vector<int>& path_nodes)
{
  std::vector<int> topo = topologicalSort();
  int numVertices = topo.size();
  std::vector<double> longest(numVertices, DBL_MIN);
  longest[startNode] = 0;

  std::vector<int> predecessors(numVertices, -1);  // Initialize with -1 (no predecessor)

  for (int i = 0; i < numVertices; i++)
  {
    int curr = topo[i];
    if (longest[curr] != DBL_MIN)
    {
      for (const auto& neighbor : adj[curr])
      {
        if (longest[neighbor.first] < longest[curr] + neighbor.second)
        {
          longest[neighbor.first] = longest[curr] + neighbor.second;
          predecessors[neighbor.first] = curr;
        }
      }
    }
  }

  path_nodes.clear();
  if (longest[endNode] != DBL_MIN)
  {
    int current = endNode;
    while (current != -1)
    {
      path_nodes.push_back(current);
      current = predecessors[current];
    }
    std::reverse(path_nodes.begin(), path_nodes.end());
  }

  for (int i = 0; i < path_nodes.size(); i++)
  {
    std::cout << path_nodes[i] << " ";
  }
  std::cout << std::endl;

  return longest[endNode];
}

std::string GetVisitedNodesString(IndexSet visited_nodes, int node_num)
{
  std::string visited_nodes_str = "";
  for (int i = 0; i < node_num; i++)
  {
    if (visited_nodes.IsElementInSet(i))
    {
      visited_nodes_str += std::to_string(i) + " ";
    }
  }
  return visited_nodes_str;
}

double DirectedAcyclicGraph::longestPath(int startNode, double startNodeProfit, std::vector<int>& path_nodes,
                                         bool accumulate_reward_at_same_node)
{
  std::vector<int> topo = topologicalSort();
  int numVertices = topo.size();
  std::vector<double> longest(numVertices, -1);
  std::vector<IndexSet> visited;
  visited.resize(numVertices, node_num_);
  longest[startNode] = startNodeProfit;
  visited[startNode].Insert(getNodeLabel(startNode));

  std::vector<int> parent_iteration_id(numVertices, -1);

  std::vector<int> predecessors(numVertices, -1);  // Initialize with -1 (no predecessor)

  /////DEBUG////////
  std::vector<std::vector<double>> all_vt;
  bool debug_save_to_file = false;

  int iteration_count = 0;

  for (int i = 0; i < numVertices; i++)
  {
    int curr_node_id = topo[i];
    int curr_node_label = getNodeLabel(curr_node_id);

    if (longest[curr_node_id] >= 0)
    {
      /////DEBUG////////
      int curr_node_t = curr_node_id % debug_t_num_;
      if (debug_save_to_file)
      {
        std::vector<double> vt;
        vt.push_back(curr_node_label);
        vt.push_back(curr_node_t);
        vt.push_back(longest[curr_node_id]);
        vt.push_back(parent_iteration_id[curr_node_id]);
        all_vt.push_back(vt);
      }

      iteration_count++;
      for (const auto& neighbor : adj[curr_node_id])
      {
        int next_node_id = neighbor.first;
        double next_node_profit = neighbor.second;

        int next_node_label = getNodeLabel(next_node_id);

        int next_node_t = next_node_id % debug_t_num_;
        // if (next_node_label == 78 && next_node_t == 65)
        // {
        //   std::cout << "curr node: " << curr_node_label << " t: " << curr_node_t << " f: " << longest[curr_node_id]
        //             << " vi: " << GetVisitedNodesString(visited[curr_node_id], node_num_) << std::endl;
        //   std::cout << "next node: " << next_node_label << " t: " << next_node_t << " f: " << longest[next_node_id]
        //             << std::endl;
        // }

        if (curr_node_label != next_node_label && visited[curr_node_id].IsElementInSet(next_node_label))
        {
          continue;
        }

        if (!accumulate_reward_at_same_node && curr_node_label == next_node_label)
        {
          next_node_profit = 0.0;
        }

        if (longest[next_node_id] < longest[curr_node_id] + next_node_profit)
        {
          longest[next_node_id] = longest[curr_node_id] + next_node_profit;
          predecessors[next_node_id] = curr_node_id;

          parent_iteration_id[next_node_id] = iteration_count;

          visited[next_node_id] = visited[curr_node_id];
          visited[next_node_id].Insert(next_node_label);

          // if (next_node_label == 78 && next_node_t == 65)
          // {
          //   std::cout << "updated next node: " << next_node_label << " t: " << next_node_t
          //             << " f: " << longest[next_node_id] << std::endl;
          // }
        }
      }
    }
  }

  std::cout << "Iteration count: " << iteration_count << std::endl;

  int endNode = 0;
  double longest_path_length = DBL_MIN;
  for (int i = 0; i < longest.size(); i++)
  {
    if (longest[i] > longest_path_length)
    {
      endNode = i;
      longest_path_length = longest[i];
    }
  }

  path_nodes.clear();
  if (longest[endNode] != DBL_MIN)
  {
    int current = endNode;
    while (current != -1)
    {
      path_nodes.push_back(current);
      current = predecessors[current];
    }
    std::reverse(path_nodes.begin(), path_nodes.end());
  }

  ///////DEBUG////////
  if (debug_save_to_file)
  {
    std::string filename = "/home/caochao/Work/ros_ws/tare_system/src/tare_planner/data/dag_vt.txt";
    misc_utils_ns::Save2DVectorToFile<double>(all_vt, filename);
  }

  return longest[endNode];
}

void DirectedAcyclicGraph::GetMarkers(visualization_msgs::MarkerArray& node_marker_array,
                                      visualization_msgs::MarkerArray& edge_marker_array)
{
  node_marker_array.markers.clear();

  visualization_msgs::Marker node_marker;
  node_marker.header.frame_id = "map";
  node_marker.header.stamp = ros::Time::now();
  node_marker.ns = "node_marker";
  node_marker.action = visualization_msgs::Marker::ADD;
  node_marker.pose.orientation.w = 1.0;
  node_marker.id = 1;
  node_marker.type = visualization_msgs::Marker::POINTS;
  node_marker.scale.x = 0.05;
  node_marker.scale.y = 0.05;
  node_marker.color.a = 0.8;
  node_marker.color.r = 0.0;
  node_marker.color.g = 1.0;
  node_marker.color.b = 0.0;

  node_marker.points.clear();

  for (int i = 0; i < positions.size(); i++)
  {
    geometry_msgs::Point node_position = positions[i];
    node_marker.points.push_back(node_position);

    visualization_msgs::Marker text_marker;
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "node_id";
    text_marker.header.frame_id = "map";
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.id = i;
    text_marker.scale.z = 0.5;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 0.0;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.text = std::to_string(i);
    node_position.x += 0.2;
    node_position.y += 0.2;
    text_marker.pose.position = node_position;

    node_marker_array.markers.push_back(text_marker);
  }

  node_marker_array.markers.push_back(node_marker);

  // Add node text

  edge_marker_array.markers.clear();
  int marker_count = 0;
  for (int i = 0; i < node_num_; i++)
  {
    int from_node_index = i;
    geometry_msgs::Point from_node_position = positions[i];
    for (const auto& edge : adj[from_node_index])
    {
      int to_node_index = edge.first;
      double weight = edge.second;

      geometry_msgs::Point to_node_position = positions[to_node_index];

      visualization_msgs::Marker arrow_marker;
      arrow_marker.header.frame_id = "map";
      arrow_marker.header.stamp = ros::Time::now();
      arrow_marker.ns = "arrow";
      arrow_marker.id = marker_count++;
      arrow_marker.type = visualization_msgs::Marker::ARROW;
      arrow_marker.action = visualization_msgs::Marker::ADD;

      arrow_marker.points.resize(2);

      arrow_marker.points[0] = from_node_position;
      arrow_marker.points[1] = to_node_position;

      arrow_marker.scale.x = 0.03;  // Shaft diameter
      arrow_marker.scale.y = 0.13;  // Head diameter
      arrow_marker.scale.z = 0.0;   // Not used

      arrow_marker.color.r = 255.0 / 255.0;  // Red
      arrow_marker.color.g = 204.0 / 255.0;  // Green
      arrow_marker.color.b = 204.0 / 255.0;  // Blue
      arrow_marker.color.a = 0.5;            // Alpha (opacity)
      arrow_marker.pose.orientation.w = 1.0;

      edge_marker_array.markers.push_back(arrow_marker);

      geometry_msgs::Point edge_mid_position;
      edge_mid_position.x = (from_node_position.x + to_node_position.x) / 2;
      edge_mid_position.y = (from_node_position.y + to_node_position.y) / 2;
      edge_mid_position.z = (from_node_position.z + to_node_position.z) / 2;
      edge_mid_position.z += 0.2;

      visualization_msgs::Marker text_marker;
      text_marker.header.stamp = ros::Time::now();
      text_marker.ns = "text";
      text_marker.header.frame_id = "map";
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.id = marker_count++;
      text_marker.scale.z = 0.3;
      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.action = visualization_msgs::Marker::ADD;

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1) << weight;
      std::string formatted_string = oss.str();

      text_marker.text = formatted_string;
      text_marker.pose.position = edge_mid_position;
      edge_marker_array.markers.push_back(text_marker);
    }
  }
}
