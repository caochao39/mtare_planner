/**
 * @file directed_acyclic_graph.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a directed acyclic graph
 * @version 0.1
 * @date 2023-04-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#include <iostream>
#include <limits.h>
#include <list>
#include <stack>
#include <vector>
#define NINF INT_MIN

#include <algorithm>
#include <utility>
#include <tuple>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <utils/TOPwTVR_utils.h>
class DirectedAcyclicGraph
{
public:
  DirectedAcyclicGraph(int numNodes)
    : node_num_(numNodes)
    , adj(numNodes)
    , dp(numNodes, DBL_MIN)
    , positions(numNodes)
    , predecessors(numNodes)
    , label_(numNodes)
  {
  }

  void addEdge(int src, int dest, double weight)
  {
    adj[src].push_back({ dest, weight });
  }

  double getEdgeWeight(int src, int dest)
  {
    if (src < 0 || src >= adj.size())
    {
      return DBL_MIN;
    }
    for (int i = 0; i < adj[src].size(); i++)
    {
      if (adj[src][i].first == dest)
      {
        return adj[src][i].second;
      }
    }
    return DBL_MIN;
  }

  void setNodePosition(int node, float x, float y, float z)
  {
    geometry_msgs::Point position;
    position.x = x;
    position.y = y;
    position.z = z;
    positions[node] = position;
  }

  geometry_msgs::Point getNodePosition(int node)
  {
    return positions[node];
  }

  void setNodeLabel(int node, int label)
  {
    label_[node] = label;
  }

  int getNodeLabel(int node)
  {
    return label_[node];
  }

  double longestPath(int startNode, int endNode, std::vector<int>& path_nodes);
  double longestPath(int startNode, double startNodeProfit, std::vector<int>& path_nodes,
                     bool accumulate_reward_at_same_node);

  void GetMarkers(visualization_msgs::MarkerArray& node_marker_array,
                  visualization_msgs::MarkerArray& edge_marker_array);

  void DebugSetTNum(int t_num)
  {
    debug_t_num_ = t_num;
  }

private:
  int node_num_;
  std::vector<int> topologicalSort();
  std::vector<std::vector<std::pair<int, double>>> adj;
  std::vector<double> dp;
  std::vector<geometry_msgs::Point> positions;
  std::vector<int> predecessors;
  std::vector<int> label_;

  int debug_t_num_;
};