/**
 * @file visualization.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that visualizes the planning process
 * @version 0.1
 * @date 2021-06-01
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils/pointcloud_utils.h"
#include "grid_world/grid_world.h"
#include "utils/misc_utils.h"
#include "coordination/multi_robot_exploration_manager.h"
#include "coordination/robot.h"

namespace tare_visualizer_ns
{
class TAREVisualizer
{
public:
  explicit TAREVisualizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  bool ReadParameters(ros::NodeHandle& nh);

  void InitializeMarkers();
  void InitializeColorList();
  void GetLocalPlanningHorizonMarker(double x, double y, double z);
  void GetGlobalSubspaceMarker(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world,
                               const std::vector<int>& ordered_cell_indices);
  void GetExploringCellMarkers(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world);
  void GetVRPPathMarkers(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world);
  void VisualizeRobotPositions(const std::vector<tare::Robot>& robots);
  void PublishMarkers();
  void PlotMobilityBoundary(const std::vector<geometry_msgs::Polygon>& mobility_boundary);
  void PlotCommsRange(tare::CommsStatus comms_status, const geometry_msgs::Point& robot_position, double comms_range);
  void PublishSyncedSubspaces(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world);
  void PublishRendezvousPosition(geometry_msgs::Point rendezvous_position);

private:
  const std::string kWorldFrameID = "map";
  bool kExploringSubspaceMarkerColorGradientAlpha;
  double kExploringSubspaceMarkerColorMaxAlpha;
  std_msgs::ColorRGBA kExploringSubspaceMarkerColor;
  std_msgs::ColorRGBA kLocalPlanningHorizonMarkerColor;
  double kLocalPlanningHorizonMarkerWidth;
  double kLocalPlanningHorizonSizeX;
  double kLocalPlanningHorizonSizeY;
  double kLocalPlanningHorizonSizeZ;
  double kGlobalSubspaceSize;
  double kGlobalSubspaceHeight;

  bool plot_mobility_boundary_;

  ros::Publisher exploring_cell_id_marker_publisher_;
  ros::Publisher synced_cell_id_markers_publisher_;
  ros::Publisher robot_id_markers_publisher_;
  ros::Publisher vrp_path_marker_publisher_;
  ros::Publisher mobility_boundary_publisher_;
  ros::Publisher uncovered_surface_point_publisher_;
  ros::Publisher viewpoint_candidate_publisher_;
  ros::Publisher viewpoint_publisher_;
  ros::Publisher local_path_publisher_;
  ros::Publisher global_path_publisher_;
  ros::Publisher rendezvous_position_publisher_;

  misc_utils_ns::Marker::Ptr global_subspaces_marker_;
  misc_utils_ns::Marker::Ptr local_planning_horizon_marker_;
  misc_utils_ns::Marker::Ptr exploring_cell_marker_;
  misc_utils_ns::Marker::Ptr comms_range_marker_;
  visualization_msgs::MarkerArray exploring_cell_id_markers_;
  visualization_msgs::MarkerArray vrp_path_markers_;
  visualization_msgs::MarkerArray mobility_boundary_markers_;
  visualization_msgs::MarkerArray synced_cell_id_markers_;
  visualization_msgs::MarkerArray robot_id_markers_;
  pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>::Ptr uncovered_surface_point_cloud_;
  pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>::Ptr viewpoint_candidate_cloud_;
  pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>::Ptr viewpoint_cloud_;
  pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>::Ptr synced_subspace_cloud_;
  pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>::Ptr robot_positions_cloud_;
  pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>::Ptr robot_in_comms_positions_cloud_;

  nav_msgs::Path local_path_;
  nav_msgs::Path global_path_;

  geometry_msgs::Point local_planning_horizon_origin_;
  geometry_msgs::Point global_subspace_origin_;
  geometry_msgs::Point global_subspace_size_;

  std::vector<std_msgs::ColorRGBA> color_list_;
  int color_list_index_;
};
}  // namespace tare_visualizer_ns