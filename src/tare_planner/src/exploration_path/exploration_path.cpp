/**
 * @file exploration_path.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements the exploration path
 * @version 0.1
 * @date 2020-10-22
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "exploration_path/exploration_path.h"

namespace exploration_path_ns
{
Node::Node()
  : type_(NodeType::LOCAL_VIA_POINT)
  , local_viewpoint_ind_(-1)
  , keypose_graph_node_ind_(-1)
  , global_subspace_index_(-1)
  , position_(Eigen::Vector3d::Zero())
  , nonstop_(false)
{
}
Node::Node(Eigen::Vector3d position) : Node()
{
  position_ = position;
}
Node::Node(geometry_msgs::Point point, NodeType type) : Node(Eigen::Vector3d(point.x, point.y, point.z))
{
  type_ = type;
}

bool Node::IsLocal()
{
  int node_type = static_cast<int>(type_);
  return node_type % 2 == 0;
}

bool operator==(const Node& n1, const Node& n2)
{
  return ((n1.position_ - n2.position_).norm() < 0.2) && (n1.type_ == n2.type_);
}

bool operator!=(const Node& n1, const Node& n2)
{
  return !(n1 == n2);
}

double ExplorationPath::GetLength() const
{
  double length = 0.0;
  if (nodes_.size() < 2)
  {
    return length;
  }
  for (int i = 1; i < nodes_.size(); i++)
  {
    length += (nodes_[i].position_ - nodes_[i - 1].position_).norm();
  }
  return length;
}

void ExplorationPath::Append(const Node& node)
{
  if (nodes_.empty() || nodes_.back() != node)
  {
    nodes_.push_back(node);
  }
}

void ExplorationPath::Append(const ExplorationPath& path)
{
  for (int i = 0; i < path.nodes_.size(); i++)
  {
    Append(path.nodes_[i]);
  }
}

void ExplorationPath::Reverse()
{
  std::reverse(nodes_.begin(), nodes_.end());
}

nav_msgs::Path ExplorationPath::GetPath() const
{
  nav_msgs::Path path;
  for (int i = 0; i < nodes_.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = nodes_[i].position_.x();
    pose.pose.position.y = nodes_[i].position_.y();
    pose.pose.position.z = nodes_[i].position_.z();
    // pose.pose.orientation.w = static_cast<int>(nodes_[i].type_);
    path.poses.push_back(pose);
  }
  return path;
}
void ExplorationPath::FromPath(const nav_msgs::Path& path)
{
  nodes_.clear();
  for (int i = 0; i < path.poses.size(); i++)
  {
    exploration_path_ns::Node node;
    node.position_.x() = path.poses[i].pose.position.x;
    node.position_.y() = path.poses[i].pose.position.y;
    node.position_.z() = path.poses[i].pose.position.z;
    node.type_ = static_cast<NodeType>(path.poses[i].pose.orientation.w);
    nodes_.push_back(node);
  }
}

void ExplorationPath::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const
{
  vis_cloud->clear();
  for (int i = 0; i < nodes_.size(); i++)
  {
    pcl::PointXYZI point;
    point.x = nodes_[i].position_.x();
    point.y = nodes_[i].position_.y();
    point.z = nodes_[i].position_.z();
    point.intensity = static_cast<int>(nodes_[i].type_);
    vis_cloud->points.push_back(point);
  }
}

void ExplorationPath::GetKeyPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const
{
  vis_cloud->clear();
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].type_ == exploration_path_ns::NodeType::ROBOT ||
        nodes_[i].type_ == exploration_path_ns::NodeType::LOOKAHEAD_POINT ||
        nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
        nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
        nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END ||
        nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT)
    {
      pcl::PointXYZI point;
      point.x = nodes_[i].position_.x();
      point.y = nodes_[i].position_.y();
      point.z = nodes_[i].position_.z();
      point.intensity = static_cast<int>(nodes_[i].type_);
      vis_cloud->points.push_back(point);
    }
  }
}

void ExplorationPath::GetNodePositions(std::vector<Eigen::Vector3d>& positions) const
{
  positions.clear();
  for (int i = 0; i < nodes_.size(); i++)
  {
    positions.push_back(nodes_[i].position_);
  }
}

int ExplorationPath::GetNodeCount(NodeType target_node_type, int start_index) const
{
  if (nodes_.empty())
  {
    return 0;
  }

  MY_ASSERT(start_index >= 0 && start_index < nodes_.size());

  int node_count = 0;
  for (int i = start_index; i < nodes_.size(); i++)
  {
    if (nodes_[i].type_ == target_node_type)
    {
      node_count++;
    }
  }
  return node_count;
}

Node ExplorationPath::GetNodeDistanceFromBegin(double distance) const
{
  double dist_from_begin = 0.0;
  for (int i = 1; i < nodes_.size(); i++)
  {
    dist_from_begin += (nodes_[i - 1].position_ - nodes_[i].position_).norm();
    if (dist_from_begin > distance)
    {
      return nodes_[i];
    }
  }
  return nodes_.back();
}

void ExplorationPath::Reset()
{
  nodes_.clear();
}

ExplorationPath ExplorationPath::Subsample(double resolution) const
{
  ExplorationPath path;
  if (nodes_.size() < 2)
  {
    path.nodes_ = nodes_;
    return path;
  }

  for (int i = 0; i < nodes_.size() - 1; i++)
  {
    std::vector<Eigen::Vector3d> interp_points;
    misc_utils_ns::LinInterpPoints(nodes_[i].position_, nodes_[i + 1].position_, resolution, interp_points);
    for (const auto& node_position : interp_points)
    {
      path.Append(Node(node_position));
    }
  }
  return path;
}

bool ExplorationPath::HasNodeType(NodeType target_node_type, int& node_index) const
{
  bool has_node_type = false;
  int node_num = nodes_.size();
  for (int i = 0; i < node_num; i++)
  {
    if (nodes_[i].type_ == target_node_type)
    {
      has_node_type = true;
      node_index = i;
      break;
    }
  }
  return has_node_type;
}

}  // namespace exploration_path_ns