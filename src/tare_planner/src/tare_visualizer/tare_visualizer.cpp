/**
 * @file tare_visualizer.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that visualizes the planning process
 * @version 0.1
 * @date 2021-06-01
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "tare_visualizer/tare_visualizer.h"
#include <chrono>
#include <thread>

namespace tare_visualizer_ns
{
TAREVisualizer::TAREVisualizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private) : plot_mobility_boundary_(false)
{
  ReadParameters(nh_private);

  exploring_cell_id_marker_publisher_ =
      nh.advertise<visualization_msgs::MarkerArray>("logging/tare_visualizer/exploring_cell_ids", 1);
  synced_cell_id_markers_publisher_ =
      nh.advertise<visualization_msgs::MarkerArray>("logging/tare_visualizer/synced_cell_ids", 1);
  robot_id_markers_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("logging/tare_visualizer/robot_ids", 1);
  vrp_path_marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("logging/tare_visualizer/vrp_paths", 1);
  mobility_boundary_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("tare_visualizer/mobility_boundary", 1);
  local_path_publisher_ = nh.advertise<nav_msgs::Path>("logging/tare_visualizer/local_path", 1);
  rendezvous_position_publisher_ =
      nh.advertise<geometry_msgs::PointStamped>("logging/tare_visualizer/rendezvous_position", 1);
  global_subspaces_marker_ =
      std::make_shared<misc_utils_ns::Marker>(nh, "logging/tare_visualizer/exploring_subspaces", kWorldFrameID);
  local_planning_horizon_marker_ =
      std::make_shared<misc_utils_ns::Marker>(nh, "logging/tare_visualizer/local_planning_horizon", kWorldFrameID);
  exploring_cell_marker_ =
      std::make_shared<misc_utils_ns::Marker>(nh, "logging/tare_visualizer/exploring_cells", kWorldFrameID);
  comms_range_marker_ =
      std::make_shared<misc_utils_ns::Marker>(nh, "logging/tare_visualizer/comms_range", kWorldFrameID);

  uncovered_surface_point_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/tare_visualizer/uncovered_surface_points", kWorldFrameID);
  viewpoint_candidate_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/tare_visualizer/viewpoint_candidates", kWorldFrameID);
  viewpoint_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/tare_visualizer/viewpoints", kWorldFrameID);
  synced_subspace_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/tare_visualizer/synced_subspaces", kWorldFrameID);
  robot_positions_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/tare_visualizer/robot_positions_cloud", kWorldFrameID);
  robot_in_comms_positions_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/tare_visualizer/robot_in_comms_positions_cloud", kWorldFrameID);

  InitializeColorList();
  InitializeMarkers();
}
bool TAREVisualizer::ReadParameters(ros::NodeHandle& nh)
{
  kExploringSubspaceMarkerColorGradientAlpha =
      misc_utils_ns::getParam<bool>(nh, "kExploringSubspaceMarkerColorGradientAlpha", true);
  kExploringSubspaceMarkerColorMaxAlpha =
      misc_utils_ns::getParam<double>(nh, "kExploringSubspaceMarkerColorMaxAlpha", 1.0);
  kExploringSubspaceMarkerColor.r = misc_utils_ns::getParam<double>(nh, "kExploringSubspaceMarkerColorR", 0.0);
  kExploringSubspaceMarkerColor.g = misc_utils_ns::getParam<double>(nh, "kExploringSubspaceMarkerColorG", 1.0);
  kExploringSubspaceMarkerColor.b = misc_utils_ns::getParam<double>(nh, "kExploringSubspaceMarkerColorB", 0.0);
  kExploringSubspaceMarkerColor.a = misc_utils_ns::getParam<double>(nh, "kExploringSubspaceMarkerColorA", 1.0);

  kLocalPlanningHorizonMarkerColor.r = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonMarkerColorR", 0.0);
  kLocalPlanningHorizonMarkerColor.g = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonMarkerColorG", 1.0);
  kLocalPlanningHorizonMarkerColor.b = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonMarkerColorB", 0.0);
  kLocalPlanningHorizonMarkerColor.a = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonMarkerColorA", 1.0);

  kLocalPlanningHorizonMarkerWidth = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonMarkerWidth", 0.3);
  int viewpoint_number = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_x", 40);
  double viewpoint_resolution = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 1.0);
  kGlobalSubspaceSize = viewpoint_number * viewpoint_resolution / 5;
  kGlobalSubspaceHeight = misc_utils_ns::getParam<double>(nh, "kGridWorldCellHeight", 3.0);

  double viewpoint_num_x = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/number_x", 35);
  double viewpoint_num_y = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/number_y", 35);
  double viewpoint_resolution_x = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 1.1);
  double viewpoint_resolution_y = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_y", 1.1);
  kLocalPlanningHorizonSizeX = viewpoint_num_x * viewpoint_resolution_x;
  kLocalPlanningHorizonSizeY = viewpoint_num_y * viewpoint_resolution_y;
  kLocalPlanningHorizonSizeZ = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonHeight", 3.0);
  return true;
}

void TAREVisualizer::InitializeMarkers()
{
  global_subspaces_marker_->SetType(visualization_msgs::Marker::CUBE_LIST);
  global_subspaces_marker_->SetScale(kGlobalSubspaceSize, kGlobalSubspaceSize, kGlobalSubspaceHeight);
  global_subspaces_marker_->SetColorRGBA(kExploringSubspaceMarkerColor);

  local_planning_horizon_marker_->SetType(visualization_msgs::Marker::LINE_LIST);
  local_planning_horizon_marker_->SetScale(kLocalPlanningHorizonMarkerWidth, 0, 0);
  local_planning_horizon_marker_->SetColorRGBA(kExploringSubspaceMarkerColor);

  exploring_cell_marker_->SetType(visualization_msgs::Marker::CUBE_LIST);
  exploring_cell_marker_->SetScale(kGlobalSubspaceSize * 0.8, kGlobalSubspaceSize * 0.8, kGlobalSubspaceHeight * 0.8);
  std_msgs::ColorRGBA kExploringSubspaceMarkerColor;
  kExploringSubspaceMarkerColor.r = 0.0;
  kExploringSubspaceMarkerColor.g = 0.0;
  kExploringSubspaceMarkerColor.b = 1.0;
  kExploringSubspaceMarkerColor.a = 0.8;
  exploring_cell_marker_->SetColorRGBA(kExploringSubspaceMarkerColor);

  comms_range_marker_->SetType(visualization_msgs::Marker::LINE_STRIP);
  comms_range_marker_->SetScale(0.3, 0, 0);
  comms_range_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);
}

void TAREVisualizer::InitializeColorList()
{
  color_list_.clear();
  color_list_.push_back(misc_utils_ns::Color(0, 255, 0));      // 0
  color_list_.push_back(misc_utils_ns::Color(255, 0, 0));      // 1
  color_list_.push_back(misc_utils_ns::Color(255, 255, 0));    // 2
  color_list_.push_back(misc_utils_ns::Color(0, 234, 255));    // 3
  color_list_.push_back(misc_utils_ns::Color(170, 0, 255));    // 4
  color_list_.push_back(misc_utils_ns::Color(255, 127, 0));    // 5
  color_list_.push_back(misc_utils_ns::Color(191, 255, 0));    // 6
  color_list_.push_back(misc_utils_ns::Color(0, 149, 255));    // 7
  color_list_.push_back(misc_utils_ns::Color(255, 0, 170));    // 8
  color_list_.push_back(misc_utils_ns::Color(255, 212, 0));    // 9
  color_list_.push_back(misc_utils_ns::Color(106, 255, 0));    // 10
  color_list_.push_back(misc_utils_ns::Color(0, 64, 255));     // 11
  color_list_.push_back(misc_utils_ns::Color(237, 185, 185));  // 12
  color_list_.push_back(misc_utils_ns::Color(185, 215, 237));  // 13
  color_list_.push_back(misc_utils_ns::Color(231, 233, 185));  // 14
  color_list_.push_back(misc_utils_ns::Color(220, 185, 237));  // 15
  color_list_.push_back(misc_utils_ns::Color(185, 237, 224));  // 16
  color_list_.push_back(misc_utils_ns::Color(143, 35, 35));    // 17
  color_list_.push_back(misc_utils_ns::Color(35, 98, 143));    // 18
  color_list_.push_back(misc_utils_ns::Color(143, 106, 35));   // 19
  color_list_.push_back(misc_utils_ns::Color(107, 35, 143));   // 20
  color_list_.push_back(misc_utils_ns::Color(79, 143, 35));    // 21
  color_list_.push_back(misc_utils_ns::Color(115, 115, 115));  // 22
  color_list_.push_back(misc_utils_ns::Color(204, 204, 204));  // 23
}

void TAREVisualizer::GetLocalPlanningHorizonMarker(double x, double y, double z)
{
  local_planning_horizon_origin_.x = x;
  local_planning_horizon_origin_.y = y;
  local_planning_horizon_origin_.z = z - kLocalPlanningHorizonSizeZ / 2;

  geometry_msgs::Point upper_right;
  upper_right.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_right.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  upper_right.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point lower_right;
  lower_right.x = local_planning_horizon_origin_.x;
  lower_right.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  lower_right.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point upper_left;
  upper_left.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_left.y = local_planning_horizon_origin_.y;
  upper_left.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point lower_left;
  lower_left.x = local_planning_horizon_origin_.x;
  lower_left.y = local_planning_horizon_origin_.y;
  lower_left.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point upper_right2;
  upper_right2.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_right2.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  upper_right2.z = local_planning_horizon_origin_.z;

  geometry_msgs::Point lower_right2;
  lower_right2.x = local_planning_horizon_origin_.x;
  lower_right2.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  lower_right2.z = local_planning_horizon_origin_.z;

  geometry_msgs::Point upper_left2;
  upper_left2.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_left2.y = local_planning_horizon_origin_.y;
  upper_left2.z = local_planning_horizon_origin_.z;

  geometry_msgs::Point lower_left2;
  lower_left2.x = local_planning_horizon_origin_.x;
  lower_left2.y = local_planning_horizon_origin_.y;
  lower_left2.z = local_planning_horizon_origin_.z;

  local_planning_horizon_marker_->marker_.points.clear();

  local_planning_horizon_marker_->marker_.points.push_back(upper_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_right);

  local_planning_horizon_marker_->marker_.points.push_back(upper_right);
  local_planning_horizon_marker_->marker_.points.push_back(upper_right2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right2);

  local_planning_horizon_marker_->marker_.points.push_back(upper_right2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_right2);
}

void TAREVisualizer::GetGlobalSubspaceMarker(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world,
                                             const std::vector<int>& ordered_cell_indices)
{
  std::vector<int> exploring_cell_indices;
  grid_world->GetExploringCellIndices(exploring_cell_indices);
  std::vector<bool> in_plan(exploring_cell_indices.size(), false);

  std::vector<std::vector<int>> ordered_cell_ids;
  grid_world->GetOrderedExploringCellIndices(ordered_cell_ids);

  int robot_cell_id = grid_world->GetRobotCellID();
  global_subspaces_marker_->marker_.points.clear();
  global_subspaces_marker_->marker_.colors.clear();
  int robot_num = ordered_cell_ids.size();
  for (int i = 0; i < robot_num; i++)
  {
    int cell_num = ordered_cell_ids[i].size() - 2;
    if (cell_num <= 0)
    {
      continue;
    }
    for (int j = 1; j < ordered_cell_ids[i].size() - 1; j++)
    {
      int cell_ind = ordered_cell_ids[i][j];
      if (!grid_world->IndInBound(cell_ind))
      {
        continue;
      }
      if (cell_ind == robot_cell_id)
      {
        continue;
      }
      if (!misc_utils_ns::ElementExistsInVector<int>(exploring_cell_indices, cell_ind))
      {
        continue;
      }

      for (int k = 0; k < exploring_cell_indices.size(); k++)
      {
        if (cell_ind == exploring_cell_indices[k])
        {
          in_plan[k] = true;
        }
      }

      geometry_msgs::Point cell_center = grid_world->GetCellPosition(cell_ind);
      std_msgs::ColorRGBA color;
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
      if (kExploringSubspaceMarkerColorGradientAlpha)
      {
        color.a = ((cell_num - j - 1) * 1.0 / cell_num) * kExploringSubspaceMarkerColorMaxAlpha;
      }
      else
      {
        color.a = 1.0;
      }
      global_subspaces_marker_->marker_.points.push_back(cell_center);
      global_subspaces_marker_->marker_.colors.push_back(color);
    }
  }

  for (int i = 0; i < exploring_cell_indices.size(); i++)
  {
    int cell_ind = exploring_cell_indices[i];
    if (in_plan[i] || !grid_world->IndInBound(cell_ind))
    {
      continue;
    }

    geometry_msgs::Point cell_center = grid_world->GetCellPosition(cell_ind);
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 0.7;
    global_subspaces_marker_->marker_.points.push_back(cell_center);
    global_subspaces_marker_->marker_.colors.push_back(color);
  }
}

void TAREVisualizer::GetExploringCellMarkers(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world)
{
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 0.5;
  for (int i = 0; i < exploring_cell_id_markers_.markers.size(); i++)
  {
    exploring_cell_id_markers_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  exploring_cell_id_marker_publisher_.publish(exploring_cell_id_markers_);

  for (int i = 0; i < synced_cell_id_markers_.markers.size(); i++)
  {
    synced_cell_id_markers_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  synced_cell_id_markers_publisher_.publish(synced_cell_id_markers_);

  exploring_cell_id_markers_.markers.clear();
  synced_cell_id_markers_.markers.clear();
  exploring_cell_marker_->marker_.points.clear();
  exploring_cell_marker_->marker_.colors.clear();

  std::vector<int> exploring_cell_ids;
  grid_world->GetExploringCellIndices(exploring_cell_ids);
  // Eigen::Vector3i robot_cell_sub = grid_world->GetRobotCellSub();
  // grid_world->GetDirectNeighborCellIndices(robot_cell_sub, exploring_cell_ids);

  int robot_num = grid_world->GetRobotNumber();
  for (int i = 0; i < exploring_cell_ids.size(); i++)
  {
    int cell_id = exploring_cell_ids[i];
    geometry_msgs::Point cell_center = grid_world->GetCellPosition(cell_id);
    exploring_cell_marker_->marker_.points.push_back(cell_center);
    exploring_cell_marker_->marker_.colors.push_back(color);
    std::string cell_label = grid_world->GetCellIDLabel(cell_id);
    visualization_msgs::Marker text_marker;
    text_marker.header.stamp = ros::Time::now();
    text_marker.header.frame_id = "map";
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.id = i;
    text_marker.scale.z = 3.0;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.text = std::to_string(robot_num + i);
    text_marker.text += "\n" + cell_label;
    text_marker.pose.position = cell_center;
    text_marker.pose.position.z += 3;
    exploring_cell_id_markers_.markers.push_back(text_marker);
  }
  exploring_cell_id_marker_publisher_.publish(exploring_cell_id_markers_);

  int cell_num = grid_world->GetCellNumber();
  for (int i = 0; i < cell_num; i++)
  {
    grid_world_ns::CellStatus cell_status = grid_world->GetCellStatus(i);
    if (cell_status == grid_world_ns::CellStatus::EXPLORING ||
        cell_status == grid_world_ns::CellStatus::EXPLORING_BY_OTHERS ||
        cell_status == grid_world_ns::CellStatus::COVERED ||
        cell_status == grid_world_ns::CellStatus::COVERED_BY_OTHERS || grid_world->IsCellCoveredToOthers(i))
    {
      geometry_msgs::Point cell_center = grid_world->GetCellPosition(i);
      std::string cell_label = std::to_string(i);
      cell_label = cell_label.substr(cell_label.size() - 4);

      std::map<int, int> exploring_synced_count = grid_world->GetCellExploringSyncedCount(i);
      std::map<int, int> explored_synced_count = grid_world->GetCellExploredSyncedCount(i);

      std::string sync_count_label = std::to_string(grid_world->GetCellSyncCount(i));

      visualization_msgs::Marker text_marker;
      text_marker.header.stamp = ros::Time::now();
      text_marker.header.frame_id = "map";
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.id = i;
      text_marker.scale.z = 3.0;
      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.action = visualization_msgs::Marker::ADD;
      text_marker.text = sync_count_label + " " + std::to_string(grid_world->GetCellUpdateID(i)) + "\n";
      text_marker.pose.position = cell_center;
      text_marker.pose.position.z += 8;
      synced_cell_id_markers_.markers.push_back(text_marker);
    }
  }
  synced_cell_id_markers_publisher_.publish(synced_cell_id_markers_);
}

void TAREVisualizer::GetVRPPathMarkers(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world)
{
  std::vector<std::vector<int>> ordered_cell_indices;
  grid_world->GetOrderedExploringCellIndices(ordered_cell_indices);

  if (vrp_path_markers_.markers.empty())
  {
    for (int i = 0; i < ordered_cell_indices.size(); i++)
    {
      visualization_msgs::Marker path_marker;
      path_marker.header.stamp = ros::Time::now();
      path_marker.header.frame_id = "map";
      path_marker.type = visualization_msgs::Marker::LINE_STRIP;
      path_marker.scale.x = 3.0;
      path_marker.pose.orientation.w = 1.0;
      path_marker.ns = "vrp_robot" + std::to_string(i);
      path_marker.id = i;
      path_marker.action = visualization_msgs::Marker::ADD;
      path_marker.color = color_list_[i];
      for (int j = 0; j < ordered_cell_indices[i].size(); j++)
      {
        int cell_ind = ordered_cell_indices[i][j];
        geometry_msgs::Point cell_position = grid_world->GetCellPosition(cell_ind);
        path_marker.points.push_back(cell_position);
      }
      path_marker.points.push_back(path_marker.points.front());
      vrp_path_markers_.markers.push_back(path_marker);
    }
  }
  else
  {
    if (ordered_cell_indices.empty())
    {
      for (int i = 0; i < vrp_path_markers_.markers.size(); i++)
      {
        vrp_path_markers_.markers[i].action = visualization_msgs::Marker::DELETE;
      }
    }
    else
    {
      for (int i = 0; i < ordered_cell_indices.size(); i++)
      {
        vrp_path_markers_.markers[i].points.clear();
        if (ordered_cell_indices[i].size() <= 2)
        {
          vrp_path_markers_.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        else
        {
          for (int j = 0; j < ordered_cell_indices[i].size(); j++)
          {
            int cell_ind = ordered_cell_indices[i][j];
            geometry_msgs::Point cell_position = grid_world->GetCellPosition(cell_ind);
            vrp_path_markers_.markers[i].points.push_back(cell_position);
          }
          vrp_path_markers_.markers[i].points.push_back(vrp_path_markers_.markers[i].points.front());
          vrp_path_markers_.markers[i].action = visualization_msgs::Marker::MODIFY;
        }
      }
    }
  }

  vrp_path_marker_publisher_.publish(vrp_path_markers_);
}

void TAREVisualizer::VisualizeRobotPositions(const std::vector<tare::Robot>& robots)
{
  bool marker_inited = !robot_id_markers_.markers.empty();
  if (!marker_inited)
  {
    for (int i = 0; i < robots.size(); i++)
    {
      visualization_msgs::Marker text_marker;
      text_marker.header.stamp = ros::Time::now();
      text_marker.header.frame_id = "map";
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.id = i;
      text_marker.scale.z = 8.0;
      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.action = visualization_msgs::Marker::ADD;
      text_marker.text = std::to_string(i);
      text_marker.pose.position = robots[i].position_;
      text_marker.pose.position.z += 1;
      robot_id_markers_.markers.push_back(text_marker);
    }
  }

  robot_positions_cloud_->cloud_->clear();
  robot_in_comms_positions_cloud_->cloud_->clear();
  for (int i = 0; i < robots.size(); i++)
  {
    geometry_msgs::Point robot_in_comms_position = robots[i].in_comms_position_;
    pcl::PointXYZI point;
    point.x = robot_in_comms_position.x;
    point.y = robot_in_comms_position.y;
    point.z = robot_in_comms_position.z;
    point.intensity = i;
    if (robots[i].lost_)
    {
      point.intensity = -1;
    }
    else
    {
      point.intensity = 1;
    }
    robot_in_comms_positions_cloud_->cloud_->points.push_back(point);

    geometry_msgs::Point robot_position = robots[i].position_;
    point.x = robot_position.x;
    point.y = robot_position.y;
    point.z = robot_position.z;
    if (robots[i].finished_exploration_)
    {
      point.intensity = 1;
    }
    else
    {
      point.intensity = -1;
    }
    robot_positions_cloud_->cloud_->points.push_back(point);

    if (marker_inited)
    {
      robot_id_markers_.markers[i].pose.position = robot_position;
      robot_id_markers_.markers[i].pose.position.z += 1;
      robot_id_markers_.markers[i].action = visualization_msgs::Marker::MODIFY;
    }
  }

  robot_in_comms_positions_cloud_->Publish();
  robot_positions_cloud_->Publish();
  robot_id_markers_publisher_.publish(robot_id_markers_);
}

void TAREVisualizer::PublishMarkers()
{
  local_planning_horizon_marker_->Publish();
  if (!global_subspaces_marker_->marker_.points.empty())
  {
    global_subspaces_marker_->SetAction(visualization_msgs::Marker::ADD);
    global_subspaces_marker_->Publish();
  }
  else
  {
    global_subspaces_marker_->SetAction(visualization_msgs::Marker::DELETE);
    global_subspaces_marker_->Publish();
  }

  if (!exploring_cell_marker_->marker_.points.empty())
  {
    exploring_cell_marker_->SetAction(visualization_msgs::Marker::ADD);
    exploring_cell_marker_->Publish();
  }
  else
  {
    exploring_cell_marker_->SetAction(visualization_msgs::Marker::DELETE);
    exploring_cell_marker_->Publish();
  }

  // for (int i = 0; i < exploring_cell_id_markers_.size(); i++)
  // {
  //   exploring_cell_id_marker_publisher_.publish(exploring_cell_id_markers_[i]);
  // }
}

void TAREVisualizer::PlotMobilityBoundary(const std::vector<geometry_msgs::Polygon>& mobility_boundary)
{
  if (plot_mobility_boundary_)
  {
    return;
  }
  plot_mobility_boundary_ = true;

  mobility_boundary_markers_.markers.clear();
  for (int i = 0; i < mobility_boundary.size(); i++)
  {
    visualization_msgs::Marker boundary_marker;
    boundary_marker.header.stamp = ros::Time::now();
    boundary_marker.header.frame_id = "map";
    boundary_marker.type = visualization_msgs::Marker::LINE_STRIP;
    boundary_marker.scale.x = 2.0;
    boundary_marker.pose.orientation.w = 1.0;
    boundary_marker.ns = "mobility_boundary";
    boundary_marker.id = i;
    boundary_marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Polygon polygon = mobility_boundary[i];
    if (polygon.points.empty())
    {
      continue;
    }
    boundary_marker.color = color_list_[static_cast<int>(polygon.points.front().z)];
    for (int j = 0; j < polygon.points.size(); j++)
    {
      geometry_msgs::Point point;
      point.x = polygon.points[j].x;
      point.y = polygon.points[j].y;
      point.z = 0;
      boundary_marker.points.push_back(point);
    }
    boundary_marker.points.push_back(boundary_marker.points.front());
    mobility_boundary_markers_.markers.push_back(boundary_marker);
  }
  mobility_boundary_publisher_.publish(mobility_boundary_markers_);
}

void TAREVisualizer::PlotCommsRange(tare::CommsStatus comms_status, const geometry_msgs::Point& robot_position,
                                    double comms_range)
{
  std_msgs::ColorRGBA color;
  switch (comms_status)
  {
    case tare::CommsStatus::IN_COMMS: {
      color = misc_utils_ns::Color(0, 255, 0);
      break;
    }
    case tare::CommsStatus::CONVOY: {
      color = misc_utils_ns::Color(0, 255, 255);
      break;
    }
    case tare::CommsStatus::RELAY_COMMS: {
      color = misc_utils_ns::Color(255, 255, 0);
      break;
    }
    case tare::CommsStatus::LOST_COMMS: {
      color = misc_utils_ns::Color(255, 0, 0);
      break;
    }
    default: {
      color = misc_utils_ns::Color(255, 255, 255);
      break;
    }
  }
  comms_range_marker_->SetColorRGBA(color);
  comms_range_marker_->marker_.points.clear();
  geometry_msgs::Point p;
  size_t circle_pnt_num = 100;
  for (size_t i = 0; i < circle_pnt_num; i++)
  {
    double theta = 2 * M_PI / circle_pnt_num * i;
    p.x = comms_range * cos(theta) + robot_position.x;
    p.y = comms_range * sin(theta) + robot_position.y;
    p.z = robot_position.z;
    comms_range_marker_->marker_.points.push_back(p);
  }
  p.x = comms_range * cos(0) + robot_position.x;
  p.y = comms_range * sin(0) + robot_position.y;
  p.z = robot_position.z;
  comms_range_marker_->marker_.points.push_back(p);

  comms_range_marker_->Publish();
}

void TAREVisualizer::PublishSyncedSubspaces(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world)
{
  synced_subspace_cloud_->cloud_->points.clear();
  int robot_num = grid_world->GetRobotNumber();
  int cell_num = grid_world->GetCellNumber();
  for (int i = 0; i < cell_num; i++)
  {
    int cell_ind = i;
    grid_world_ns::CellStatus cell_status = grid_world->GetCellStatus(cell_ind);
    if (cell_status == grid_world_ns::CellStatus::EXPLORING ||
        cell_status == grid_world_ns::CellStatus::EXPLORING_BY_OTHERS ||
        cell_status == grid_world_ns::CellStatus::COVERED ||
        cell_status == grid_world_ns::CellStatus::COVERED_BY_OTHERS || grid_world->IsCellCoveredToOthers(cell_ind))
    {
      geometry_msgs::Point cell_center = grid_world->GetCellPosition(cell_ind);
      if (grid_world->IsCellSyncedByRobot(cell_ind, 0) || grid_world->IsCellSyncedByRobot(cell_ind, 1) ||
          grid_world->IsCellSyncedByRobot(cell_ind, 2))
      {
        pcl::PointXYZI point;
        point.x = cell_center.x;
        point.y = cell_center.y;
        point.z = cell_center.z + 5;
        point.intensity = grid_world->GetCellSyncedRobotIDs(cell_ind);
        synced_subspace_cloud_->cloud_->points.push_back(point);
      }
    }
  }
  synced_subspace_cloud_->Publish();
}

void TAREVisualizer::PublishRendezvousPosition(geometry_msgs::Point rendezvous_position)
{
  geometry_msgs::PointStamped point_stamped;
  point_stamped.header.frame_id = kWorldFrameID;
  point_stamped.header.stamp = ros::Time::now();
  point_stamped.point = rendezvous_position;
  rendezvous_position_publisher_.publish(point_stamped);
}

}  // namespace tare_visualizer_ns
