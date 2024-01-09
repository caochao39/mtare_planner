//
// Created by caochao on 12/31/19.
//

#ifndef SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H
#define SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H

#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <planning_env/planning_env.h>
#include <utils/misc_utils.h>

namespace viewpoint_manager_ns
{
class ViewPointManager;
}

namespace keypose_graph_ns
{
struct KeyposeNode;
class KeyposeGraph;
const double INF = 9999.0;
typedef std::pair<int, int> iPair;
}  // namespace keypose_graph_ns

struct keypose_graph_ns::KeyposeNode
{
  int node_ind_;
  int keypose_id_;
  int cell_ind_;
  bool is_keypose_;
  bool is_connected_;
  geometry_msgs::Point position_;
  geometry_msgs::Point offset_to_keypose_;

public:
  explicit KeyposeNode(double x = 0, double y = 0, double z = 0, int node_ind = 0, bool is_keypose = true);
  explicit KeyposeNode(const geometry_msgs::Point& point, int node_ind = 0, bool is_keypose = true);
  ~KeyposeNode() = default;
  bool IsKeypose() const
  {
    return is_keypose_;
  }
  bool IsConnected() const
  {
    return is_connected_;
  }
  void SetOffsetToKeypose(const geometry_msgs::Point& offset_to_keypose)
  {
    offset_to_keypose_ = offset_to_keypose;
  }
  void SetCurrentKeyposePosition(const geometry_msgs::Point& current_keypose_position)
  {
    offset_to_keypose_.x = position_.x - current_keypose_position.x;
    offset_to_keypose_.y = position_.y - current_keypose_position.y;
    offset_to_keypose_.z = position_.z - current_keypose_position.z;
  }
  void SetCurrentKeyposeID(int keypose_id)
  {
    keypose_id_ = keypose_id;
  }

  friend std::ostream& operator<<(std::ostream& out, const KeyposeNode& node);
  friend std::istream& operator>>(std::istream& in, KeyposeNode& node);
};

class keypose_graph_ns::KeyposeGraph
{
private:
  bool allow_vertical_edge_;
  int current_keypose_id_;
  geometry_msgs::Point current_keypose_position_;
  std::vector<std::vector<int>> graph_;
  std::vector<std::vector<double>> dist_;
  std::vector<bool> in_local_planning_horizon_;
  std::vector<KeyposeNode> nodes_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_connected_nodes_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr connected_nodes_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_nodes_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr nodes_cloud_;

  std::vector<int> connected_node_indices_;

  double kAddNodeMinDist;
  double kAddNonKeyposeNodeMinDist;
  double kAddEdgeConnectDistThr;
  double kAddEdgeToLastKeyposeDistThr;
  double kAddEdgeVerticalThreshold;
  double kAddEdgeCollisionCheckResolution;
  double kAddEdgeCollisionCheckRadius;
  int kAddEdgeCollisionCheckPointNumThr;

  static bool ComparePair(const std::pair<int, int>& a, const std::pair<int, int>& b)
  {
    return (a.first == b.first && a.second == b.second) || (a.first == b.second && a.second == b.first);
  }

public:
  KeyposeGraph(ros::NodeHandle& nh);
  ~KeyposeGraph() = default;
  void ReadParameters(ros::NodeHandle& nh);
  void LoadFromFile(std::string nodes_filename, std::string edges_filename);
  void SetCurrentKeypose(const nav_msgs::Odometry& keypose);
  void AddNode(const geometry_msgs::Point& position, int node_ind, bool is_keypose);
  void AddNodeAndEdge(const geometry_msgs::Point& position, int node_ind, bool is_keypose, int connected_node_ind,
                      double connected_node_dist);
  void AddEdge(int from_node_ind, int to_node_ind, double dist);
  bool HasNode(const Eigen::Vector3d& position);
  bool IsNodeKeypose(int node_ind)
  {
    if (InBound(node_ind))
    {
      return nodes_[node_ind].is_keypose_;
    }
    return false;
  }
  bool InBound(int index)
  {
    return index >= 0 && index < graph_.size();
  }
  int GetNodeCount()
  {
    return nodes_.size();
  }

  int GetKeyposeNodeCount();
  int GetConnectedNodeCount();
  void GetNodes(std::vector<KeyposeNode>& nodes);
  void GetEdges(std::vector<std::vector<int>>& edges);
  void GetMarker(visualization_msgs::Marker& node_marker, visualization_msgs::Marker& edge_marker);
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  std::vector<int> GetConnectedGraphNodeIndices()
  {
    return connected_node_indices_;
  }
  void GetConnectedNodeIndices(int query_ind, std::vector<int>& connected_node_indices, std::vector<bool> constraints);
  void CheckLocalCollision(const geometry_msgs::Point& robot_position,
                           const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);
  void UpdateNodes();
  void CheckConnectivity(const geometry_msgs::Point& robot_position);
  int AddKeyposeNode(const geometry_msgs::Point& keypose_position, const planning_env_ns::PlanningEnv& planning_env,
                     const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);
  bool HasEdgeBetween(int node_ind1, int node_ind2);
  bool IsConnected(const Eigen::Vector3d& from_position, const Eigen::Vector3d& to_position);
  int AddNonKeyposeNode(const geometry_msgs::Point& new_node_position);
  void AddPath(const nav_msgs::Path& path);
  void SetAllowVerticalEdge(bool allow_vertical_edge)
  {
    allow_vertical_edge_ = allow_vertical_edge;
  }
  bool IsPositionReachable(const geometry_msgs::Point& point, double dist_threshold);
  bool IsPositionReachable(const geometry_msgs::Point& point);
  int GetClosestNodeInd(const geometry_msgs::Point& point);
  void GetClosestNodeIndAndDistance(const geometry_msgs::Point& point, int& node_ind, double& dist);
  void GetClosestConnectedNodeIndAndDistance(const geometry_msgs::Point& point, int& node_ind, double& dist);
  int GetClosestKeyposeID(const geometry_msgs::Point& point);
  geometry_msgs::Point GetClosestNodePosition(const geometry_msgs::Point& point);
  bool GetShortestPathWithMaxLength(const geometry_msgs::Point& start_point, const geometry_msgs::Point& target_point,
                                    double max_path_length, bool get_path, nav_msgs::Path& path);
  double GetShortestPath(const geometry_msgs::Point& start_point, const geometry_msgs::Point& target_point,
                         bool get_path, nav_msgs::Path& path, bool use_connected_nodes = false);

  double& SetAddNodeMinDist()
  {
    return kAddNodeMinDist;
  }
  double& SetAddNonKeyposeNodeMinDist()
  {
    return kAddNonKeyposeNodeMinDist;
  }
  double& SetAddEdgeCollisionCheckResolution()
  {
    return kAddEdgeCollisionCheckResolution;
  }
  double& SetAddEdgeCollisionCheckRadius()
  {
    return kAddEdgeCollisionCheckRadius;
  }
  int& SetAddEdgeCollisionCheckPointNumThr()
  {
    return kAddEdgeCollisionCheckPointNumThr;
  }
  double& SetAddEdgeConnectDistThr()
  {
    return kAddEdgeConnectDistThr;
  }
  double& SetAddEdgeToLastKeyposeDistThr()
  {
    return kAddEdgeToLastKeyposeDistThr;
  }
  double& SetAddEdgeVerticalThreshold()
  {
    return kAddEdgeVerticalThreshold;
  }
  geometry_msgs::Point GetFirstKeyposePosition();
  geometry_msgs::Point GetKeyposePosition(int keypose_id);
  void GetKeyposePositions(std::vector<Eigen::Vector3d>& positions);
  geometry_msgs::Point GetNodePosition(int node_ind);
  void UpdateNodePositionWithLoopClosure(const nav_msgs::Path& keypose_path);
};

#endif  // SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H
