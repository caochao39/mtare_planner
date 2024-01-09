/**
 * @file sensor_coverage_planner_ground.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <ros/package.h>
#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"
#include <tare_msgs/ExplorationInfo.h>

namespace sensor_coverage_planner_3d_ns
{
bool PlannerParameters::ReadParameters(ros::NodeHandle& nh)
{
  sub_start_exploration_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_start_exploration_topic_", "/exploration_start");
  sub_keypose_topic_ = misc_utils_ns::getParam<std::string>(nh, "sub_keypose_topic_", "/key_pose_to_map");
  sub_state_estimation_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_state_estimation_topic_", "/state_estimation_at_scan");
  sub_registered_scan_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_registered_scan_topic_", "/registered_scan");
  sub_terrain_map_topic_ = misc_utils_ns::getParam<std::string>(nh, "sub_terrain_map_topic_", "/terrain_map");
  sub_terrain_map_ext_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_terrain_map_ext_topic_", "/terrain_map_ext");
  sub_coverage_boundary_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_coverage_boundary_topic_", "/coverage_boundary");
  sub_viewpoint_boundary_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_viewpoint_boundary_topic_", "/navigation_boundary");
  sub_nogo_boundary_topic_ = misc_utils_ns::getParam<std::string>(nh, "sub_nogo_boundary_topic_", "/nogo_boundary");
  sub_mobility_boundary_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_mobility_boundary_topic_", "/mobility_boundary");
  pub_exploration_finish_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_exploration_finish_topic_", "exploration_finish");
  pub_runtime_breakdown_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_runtime_breakdown_topic_", "runtime_breakdown");
  pub_runtime_topic_ = misc_utils_ns::getParam<std::string>(nh, "pub_runtime_topic_", "/runtime");
  pub_waypoint_topic_ = misc_utils_ns::getParam<std::string>(nh, "pub_waypoint_topic_", "/way_point");
  pub_momentum_activation_count_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_momentum_activation_count_topic_", "momentum_activation_count");
  sub_virtual_obstacle_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_virtual_obstacle_topic_", "/virtual_obstacle");
  pub_exploration_info_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_exploration_info_topic_", "/exploration_info");

  pub_robot_position_topic_ = misc_utils_ns::getParam<std::string>(nh, "pub_robot_position_topic_", "/position");
  pub_comms_relay_topic_ = misc_utils_ns::getParam<std::string>(nh, "pub_comms_relay_topic_", "relay_comms");

  // Bool
  kAutoStart = misc_utils_ns::getParam<bool>(nh, "kAutoStart", false);
  kRushHome = misc_utils_ns::getParam<bool>(nh, "kRushHome", false);
  kUseTerrainHeight = misc_utils_ns::getParam<bool>(nh, "kUseTerrainHeight", true);
  kCheckTerrainCollision = misc_utils_ns::getParam<bool>(nh, "kCheckTerrainCollision", true);
  kCheckVirtualObstacleCollision = misc_utils_ns::getParam<bool>(nh, "kCheckVirtualObstacleCollision", true);
  kExtendWayPoint = misc_utils_ns::getParam<bool>(nh, "kExtendWayPoint", true);
  kUseLineOfSightLookAheadPoint = misc_utils_ns::getParam<bool>(nh, "kUseLineOfSightLookAheadPoint", true);
  kNoExplorationReturnHome = misc_utils_ns::getParam<bool>(nh, "kNoExplorationReturnHome", true);
  kUseMomentum = misc_utils_ns::getParam<bool>(nh, "kUseMomentum", false);
  // TODO: use grid_world_'s kUseTimeBudget value to avoid inconsistency
  kUseTimeBudget = misc_utils_ns::getParam<bool>(nh, "kUseTimeBudget", false);

  // Double
  kKeyposeCloudDwzFilterLeafSize = misc_utils_ns::getParam<double>(nh, "kKeyposeCloudDwzFilterLeafSize", 0.2);
  kRushHomeDist = misc_utils_ns::getParam<double>(nh, "kRushHomeDist", 10.0);
  kAtHomeDistThreshold = misc_utils_ns::getParam<double>(nh, "kAtHomeDistThreshold", 0.5);
  kTerrainCollisionThreshold = misc_utils_ns::getParam<double>(nh, "kTerrainCollisionThreshold", 0.5);
  kLookAheadDistance = misc_utils_ns::getParam<double>(nh, "kLookAheadDistance", 5.0);
  kExtendWayPointDistanceBig = misc_utils_ns::getParam<double>(nh, "kExtendWayPointDistanceBig", 8.0);
  kExtendWayPointDistanceSmall = misc_utils_ns::getParam<double>(nh, "kExtendWayPointDistanceSmall", 3.0);
  kAddKeyposeNodeMinDist = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddNodeMinDist", 1.0);

  // Int
  kDirectionChangeCounterThr = misc_utils_ns::getParam<int>(nh, "kDirectionChangeCounterThr", 4);
  kDirectionNoChangeCounterThr = misc_utils_ns::getParam<int>(nh, "kDirectionNoChangeCounterThr", 5);

  return true;
}

void PlannerData::Initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p)
{
  keypose_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "keypose_cloud", kWorldFrameID);
  registered_scan_stack_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>(nh, "registered_scan_stack", kWorldFrameID);
  registered_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "registered_cloud", kWorldFrameID);
  large_terrain_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_cloud_large", kWorldFrameID);
  terrain_collision_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_collision_cloud", kWorldFrameID);
  virtual_obstacle_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "virtual_obstacle_cloud", kWorldFrameID);
  terrain_ext_collision_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_ext_collision_cloud", kWorldFrameID);
  viewpoint_vis_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "logging/viewpoint_vis_cloud", kWorldFrameID);
  grid_world_vis_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/grid_world_vis_cloud", kWorldFrameID);
  low_priority_exploring_cell_vis_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/low_priority_exploring_cell_vis_cloud", kWorldFrameID);
  exploration_path_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/exploration_path_cloud", kWorldFrameID);
  global_path_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "logging/global_path_cloud", kWorldFrameID);

  selected_viewpoint_vis_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/selected_viewpoint_vis_cloud", kWorldFrameID);
  exploring_cell_vis_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/exploring_cell_vis_cloud", kWorldFrameID);
  collision_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "logging/collision_cloud", kWorldFrameID);
  lookahead_point_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "logging/lookahead_point_cloud", kWorldFrameID);
  keypose_graph_vis_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "logging/keypose_graph_cloud", kWorldFrameID);
  viewpoint_in_collision_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "viewpoint_in_collision_cloud_", kWorldFrameID);
  point_cloud_manager_neighbor_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "pointcloud_manager_cloud", kWorldFrameID);
  reordered_global_subspace_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "reordered_global_subspace_cloud", kWorldFrameID);
  subsampled_global_path_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "subsampled_global_path_cloud_", kWorldFrameID);

  planning_env_ = std::make_unique<planning_env_ns::PlanningEnv>(nh, nh_p);
  viewpoint_manager_ = std::make_shared<viewpoint_manager_ns::ViewPointManager>(nh_p);
  local_coverage_planner_ = std::make_unique<local_coverage_planner_ns::LocalCoveragePlanner>(nh_p);
  local_coverage_planner_->SetViewPointManager(viewpoint_manager_);
  keypose_graph_ = std::make_unique<keypose_graph_ns::KeyposeGraph>(nh_p);
  grid_world_ = std::make_unique<grid_world_ns::GridWorld>(nh_p);
  grid_world_->SetUseKeyposeGraph(true);
  visualizer_ = std::make_unique<tare_visualizer_ns::TAREVisualizer>(nh, nh_p);
  multi_robot_exploration_info_manager_ = std::make_unique<tare::MultiRobotExplorationManager>(nh, nh_p);
  grid_world_->SetSelfMobilityType(multi_robot_exploration_info_manager_->GetSelfType());

  initial_position_.x() = 0.0;
  initial_position_.y() = 0.0;
  initial_position_.z() = 0.0;

  cur_keypose_node_ind_ = 0;

  keypose_graph_node_marker_ =
      std::make_unique<misc_utils_ns::Marker>(nh, "logging/keypose_graph_node_marker", kWorldFrameID);
  keypose_graph_node_marker_->SetType(visualization_msgs::Marker::POINTS);
  keypose_graph_node_marker_->SetScale(0.4, 0.4, 0.1);
  keypose_graph_node_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);
  keypose_graph_edge_marker_ =
      std::make_unique<misc_utils_ns::Marker>(nh, "logging/keypose_graph_edge_marker", kWorldFrameID);
  keypose_graph_edge_marker_->SetType(visualization_msgs::Marker::LINE_LIST);
  keypose_graph_edge_marker_->SetScale(0.05, 0.0, 0.0);
  keypose_graph_edge_marker_->SetColorRGBA(1.0, 1.0, 0.0, 0.9);

  nogo_boundary_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "nogo_boundary_marker", kWorldFrameID);
  nogo_boundary_marker_->SetType(visualization_msgs::Marker::LINE_LIST);
  nogo_boundary_marker_->SetScale(0.05, 0.0, 0.0);
  nogo_boundary_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  grid_world_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "logging/grid_world_marker", kWorldFrameID);
  grid_world_marker_->SetType(visualization_msgs::Marker::CUBE_LIST);
  grid_world_marker_->SetScale(1.0, 1.0, 1.0);
  grid_world_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  global_roadmap_node_marker_ =
      std::make_unique<misc_utils_ns::Marker>(nh, "logging/global_roadmap_node_marker", kWorldFrameID);
  global_roadmap_node_marker_->SetType(visualization_msgs::Marker::POINTS);
  global_roadmap_node_marker_->SetScale(2.0, 2.0, 2.0);
  global_roadmap_node_marker_->SetColorRGBA(0.0, 0.0, 1.0, 0.8);

  global_roadmap_edge_marker_ =
      std::make_unique<misc_utils_ns::Marker>(nh, "logging/global_roadmap_edge_marker", kWorldFrameID);
  global_roadmap_edge_marker_->SetType(visualization_msgs::Marker::LINE_LIST);
  global_roadmap_edge_marker_->SetScale(0.3, 0.3, 0.3);
  // global_roadmap_edge_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  robot_yaw_ = 0.0;
  lookahead_point_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_forward_ = true;

  Eigen::Vector3d viewpoint_resolution = viewpoint_manager_->GetResolution();
  double add_non_keypose_node_min_dist = std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
  keypose_graph_->SetAddNonKeyposeNodeMinDist() = add_non_keypose_node_min_dist;

  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;

  last_robot_position_ = robot_position_;

  manual_waypoint_ = robot_position_;

  // tmp
  // grid_world_->LoadRoadmapFromFile();
}

SensorCoveragePlanner3D::SensorCoveragePlanner3D(ros::NodeHandle& nh, ros::NodeHandle& nh_p)
  : keypose_cloud_update_(false)
  , initialized_(false)
  , first_initialization_(false)
  , lookahead_point_update_(false)
  , relocation_(false)
  , start_exploration_(false)
  , exploration_finished_(false)
  , others_finished_exploring_(false)
  , near_home_(false)
  , at_home_(false)
  , stopped_(false)
  , test_point_update_(false)
  , viewpoint_ind_update_(false)
  , step_(false)
  , use_momentum_(false)
  , lookahead_point_in_line_of_sight_(true)
  , pause_(false)
  , follow_manual_waypoint_(false)
  , convoy_(false)
  , relay_comms_(false)
  , registered_cloud_count_(0)
  , keypose_count_(0)
  , direction_change_count_(0)
  , direction_no_change_count_(0)
  , momentum_activation_count_(0)
  , next_global_subspace_index_(-1)
  , free_paths_empty_count_(0)
{
  initialize(nh, nh_p);
  PrintExplorationStatus("Exploration Started", false);
}

bool SensorCoveragePlanner3D::initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p)
{
  if (!pp_.ReadParameters(nh_p))
  {
    ROS_ERROR("Read parameters failed");
    return false;
  }

  pd_.Initialize(nh, nh_p);

  id_ = pd_.multi_robot_exploration_info_manager_->GetSelfID();

  pd_.keypose_graph_->SetAllowVerticalEdge(false);

  lidar_model_ns::LiDARModel::setCloudDWZResol(pd_.planning_env_->GetPlannerCloudResolution());

  execution_timer_ = nh.createTimer(ros::Duration(1.0), &SensorCoveragePlanner3D::execute, this);

  exploration_start_sub_ =
      nh.subscribe(pp_.sub_start_exploration_topic_, 5, &SensorCoveragePlanner3D::ExplorationStartCallback, this);
  key_pose_sub_ = nh.subscribe(pp_.sub_keypose_topic_, 5, &SensorCoveragePlanner3D::KeyposeCallback, this);
  registered_scan_sub_ =
      nh.subscribe(pp_.sub_registered_scan_topic_, 5, &SensorCoveragePlanner3D::RegisteredScanCallback, this);
  terrain_map_sub_ = nh.subscribe(pp_.sub_terrain_map_topic_, 5, &SensorCoveragePlanner3D::TerrainMapCallback, this);
  terrain_map_ext_sub_ =
      nh.subscribe(pp_.sub_terrain_map_ext_topic_, 5, &SensorCoveragePlanner3D::TerrainMapExtCallback, this);
  state_estimation_sub_ =
      nh.subscribe(pp_.sub_state_estimation_topic_, 5, &SensorCoveragePlanner3D::StateEstimationCallback, this);
  coverage_boundary_sub_ =
      nh.subscribe(pp_.sub_coverage_boundary_topic_, 1, &SensorCoveragePlanner3D::CoverageBoundaryCallback, this);
  viewpoint_boundary_sub_ =
      nh.subscribe(pp_.sub_viewpoint_boundary_topic_, 1, &SensorCoveragePlanner3D::ViewPointBoundaryCallback, this);
  nogo_boundary_sub_ =
      nh.subscribe(pp_.sub_nogo_boundary_topic_, 1, &SensorCoveragePlanner3D::NogoBoundaryCallback, this);
  mobility_boundary_sub_ =
      nh.subscribe(pp_.sub_mobility_boundary_topic_, 1, &SensorCoveragePlanner3D::MobilityBoundaryCallback, this);
  virtual_obstacle_sub_ =
      nh.subscribe(pp_.sub_virtual_obstacle_topic_, 1, &SensorCoveragePlanner3D::VirtualObstacleCallback, this);
  stuck_point_sub_ = covered_point_sub_ =
      nh.subscribe(pp_.sub_covered_point_topic_, 1, &SensorCoveragePlanner3D::CoveredPointCallback, this);
  unexplored_point_sub_ =
      nh.subscribe(pp_.sub_unexplored_point_topic_, 1, &SensorCoveragePlanner3D::UnexploredPointCallback, this);
  aggressive_sub_ = nh.subscribe("/aggressive_mode", 1, &SensorCoveragePlanner3D::PlannerAggressiveCallback, this);
  pause_sub_ = nh.subscribe("/pause_tare", 1, &SensorCoveragePlanner3D::PauseCallback, this);
  nav_goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &SensorCoveragePlanner3D::NavGoalCallback, this);
  initial_pose_sub_ = nh.subscribe("/initialpose", 1, &SensorCoveragePlanner3D::InitialPoseCallback, this);
  free_paths_sub_ = nh.subscribe("/free_paths", 1, &SensorCoveragePlanner3D::FreePathsCallback, this);

  global_path_full_publisher_ = nh.advertise<nav_msgs::Path>("logging/global_path_full", 1);
  global_path_publisher_ = nh.advertise<nav_msgs::Path>("logging/global_path", 1);
  local_tsp_path_publisher_ = nh.advertise<nav_msgs::Path>("logging/local_path", 1);
  exploration_path_publisher_ = nh.advertise<nav_msgs::Path>("logging/exploration_path", 1);
  waypoint_pub_ = nh.advertise<geometry_msgs::PointStamped>(pp_.pub_waypoint_topic_, 2);
  exploration_finish_pub_ = nh.advertise<std_msgs::Bool>(pp_.pub_exploration_finish_topic_, 2);
  runtime_breakdown_pub_ = nh.advertise<std_msgs::Int32MultiArray>(pp_.pub_runtime_breakdown_topic_, 2);
  runtime_pub_ = nh.advertise<std_msgs::Float32>(pp_.pub_runtime_topic_, 2);
  momentum_activation_count_pub_ = nh.advertise<std_msgs::Int32>(pp_.pub_momentum_activation_count_topic_, 2);
  planner_aggressiveness_pub_ = nh.advertise<std_msgs::Bool>("aggressive", 1);
  global_comms_relay_nav_path_publisher_ = nh.advertise<nav_msgs::Path>("logging/global_comms_relay_path", 1);
  local_comms_relay_nav_path_publisher_ = nh.advertise<nav_msgs::Path>("logging/local_comms_relay_path", 1);
  relay_comms_publisher_ = nh.advertise<std_msgs::Bool>(pp_.pub_comms_relay_topic_, 1);
  vrp_longest_route_length_publisher_ = nh.advertise<std_msgs::Int32>("vrp_longest_route_length", 1);
  map_clearing_publisher_ = nh.advertise<std_msgs::Float32>("/map_clearing", 1);

  // Coordination

  pp_.pub_exploration_info_topic_ =
      "/" + pd_.multi_robot_exploration_info_manager_->GetSelfName() + pp_.pub_exploration_info_topic_;
  exploration_info_pub_ = nh.advertise<tare_msgs::ExplorationInfo>(pp_.pub_exploration_info_topic_, 2);

  pp_.pub_robot_position_topic_ =
      "/" + pd_.multi_robot_exploration_info_manager_->GetSelfName() + pp_.pub_robot_position_topic_;
  robot_position_pub_ = nh.advertise<geometry_msgs::PointStamped>(pp_.pub_robot_position_topic_, 2);

  std::string pub_robot_exploration_finish_topic =
      "/" + pd_.multi_robot_exploration_info_manager_->GetSelfName() + "/exploration_finish";
  robot_exploration_finish_pub_ = nh.advertise<std_msgs::Bool>(pub_robot_exploration_finish_topic, 2);

  // Debug
  pointcloud_manager_neighbor_cells_origin_pub_ =
      nh.advertise<geometry_msgs::PointStamped>("pointcloud_manager_neighbor_cells_origin", 1);

  return true;
}

void SensorCoveragePlanner3D::ExplorationStartCallback(const std_msgs::Bool::ConstPtr& start_msg)
{
  if (start_msg->data)
  {
    start_exploration_ = true;
  }
}

void SensorCoveragePlanner3D::StateEstimationCallback(const nav_msgs::Odometry::ConstPtr& state_estimation_msg)
{
  pd_.robot_position_ = state_estimation_msg->pose.pose.position;
  // Todo: use a boolean
  if (std::abs(pd_.initial_position_.x()) < 0.01 && std::abs(pd_.initial_position_.y()) < 0.01 &&
      std::abs(pd_.initial_position_.z()) < 0.01)
  {
    pd_.initial_position_.x() = pd_.robot_position_.x;
    pd_.initial_position_.y() = pd_.robot_position_.y;
    pd_.initial_position_.z() = pd_.robot_position_.z;
    pd_.last_added_keypose_position_ = pd_.robot_position_;
  }
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geo_quat = state_estimation_msg->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);

  pd_.robot_yaw_ = yaw;

  if (state_estimation_msg->twist.twist.linear.x > 0.4)
  {
    pd_.moving_forward_ = true;
  }
  else if (state_estimation_msg->twist.twist.linear.x < -0.4)
  {
    pd_.moving_forward_ = false;
  }

  initialized_ = true;
}

void SensorCoveragePlanner3D::KeyposeCallback(const nav_msgs::Odometry::ConstPtr& keypose_msg)
{
  pd_.keypose_ = *keypose_msg;
  misc_utils_ns::LeftRotatePoint(pd_.keypose_.pose.pose.position);
  pd_.keypose_graph_->SetCurrentKeypose(pd_.keypose_);
}

void SensorCoveragePlanner3D::KeyPosePathCallback(const nav_msgs::Path::ConstPtr& keypose_path_msg)
{
  nav_msgs::Path keypose_path = *keypose_path_msg;
  for (int i = 0; i < keypose_path.poses.size(); i++)
  {
    misc_utils_ns::LeftRotatePoint(keypose_path.poses[i].pose.position);
  }
  pd_.keypose_graph_->UpdateNodePositionWithLoopClosure(keypose_path);
}

void SensorCoveragePlanner3D::RegisteredScanCallback(const sensor_msgs::PointCloud2ConstPtr& registered_scan_msg)
{
  if (!initialized_)
  {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_tmp(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*registered_scan_msg, *registered_scan_tmp);
  if (registered_scan_tmp->points.empty())
  {
    return;
  }

  // For using LOAM interfaces
  // RotatePointCloud<pcl::PointXYZ>(registered_scan_tmp);

  *(pd_.registered_scan_stack_->cloud_) += *(registered_scan_tmp);
  pointcloud_downsizer_.Downsize(registered_scan_tmp, pp_.kKeyposeCloudDwzFilterLeafSize,
                                 pp_.kKeyposeCloudDwzFilterLeafSize, pp_.kKeyposeCloudDwzFilterLeafSize);
  pd_.registered_cloud_->cloud_->clear();
  pcl::copyPointCloud(*registered_scan_tmp, *(pd_.registered_cloud_->cloud_));

  pd_.planning_env_->UpdateRobotPosition(pd_.robot_position_);
  pd_.planning_env_->UpdateRegisteredCloud<pcl::PointXYZI>(pd_.registered_cloud_->cloud_);

  registered_cloud_count_ = (registered_cloud_count_ + 1) % 5;
  if (registered_cloud_count_ == 0)
  {
    pointcloud_downsizer_.Downsize(pd_.registered_scan_stack_->cloud_, pp_.kKeyposeCloudDwzFilterLeafSize,
                                   pp_.kKeyposeCloudDwzFilterLeafSize, pp_.kKeyposeCloudDwzFilterLeafSize);

    pd_.keypose_cloud_->cloud_->clear();
    pcl::copyPointCloud(*(pd_.registered_scan_stack_->cloud_), *(pd_.keypose_cloud_->cloud_));
    // pd_.keypose_cloud_->Publish();
    pd_.registered_scan_stack_->cloud_->clear();
    keypose_cloud_update_ = true;
  }
}

void SensorCoveragePlanner3D::TerrainMapCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_map_msg)
{
  if (pp_.kCheckTerrainCollision)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_map_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_msg, *terrain_map_tmp);

    // For using LOAM interfaces
    // RotatePointCloud<pcl::PointXYZI>(terrain_map_tmp);

    pd_.terrain_collision_cloud_->cloud_->clear();
    for (auto& point : terrain_map_tmp->points)
    {
      if (point.intensity > pp_.kTerrainCollisionThreshold)
      {
        pd_.terrain_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void SensorCoveragePlanner3D::TerrainMapExtCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_map_ext_msg)
{
  if (pp_.kUseTerrainHeight)
  {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg, *(pd_.large_terrain_cloud_->cloud_));

    // For using LOAM interfaces
    // RotatePointCloud<pcl::PointXYZI>(pd_.large_terrain_cloud_->cloud_);
  }
  if (pp_.kCheckTerrainCollision)
  {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg, *(pd_.large_terrain_cloud_->cloud_));

    // For using LOAM interfaces
    // RotatePointCloud<pcl::PointXYZI>(pd_.large_terrain_cloud_->cloud_);

    pd_.terrain_ext_collision_cloud_->cloud_->clear();
    for (auto& point : pd_.large_terrain_cloud_->cloud_->points)
    {
      if (point.intensity > pp_.kTerrainCollisionThreshold)
      {
        pd_.terrain_ext_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void SensorCoveragePlanner3D::CoverageBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg)
{
  pd_.planning_env_->UpdateCoverageBoundary((*polygon_msg).polygon);
}

void SensorCoveragePlanner3D::ViewPointBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg)
{
  pd_.viewpoint_manager_->UpdateViewPointBoundary((*polygon_msg).polygon);
}

void SensorCoveragePlanner3D::NogoBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg)
{
  if (polygon_msg->polygon.points.empty())
  {
    return;
  }
  double polygon_id = polygon_msg->polygon.points[0].z;
  int polygon_point_size = polygon_msg->polygon.points.size();
  std::vector<geometry_msgs::Polygon> nogo_boundary;
  geometry_msgs::Polygon polygon;
  for (int i = 0; i < polygon_point_size; i++)
  {
    if (polygon_msg->polygon.points[i].z == polygon_id)
    {
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    }
    else
    {
      nogo_boundary.push_back(polygon);
      polygon.points.clear();
      polygon_id = polygon_msg->polygon.points[i].z;
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    }
  }
  nogo_boundary.push_back(polygon);
  pd_.viewpoint_manager_->UpdateNogoBoundary(nogo_boundary);

  geometry_msgs::Point point;
  for (int i = 0; i < nogo_boundary.size(); i++)
  {
    for (int j = 0; j < nogo_boundary[i].points.size() - 1; j++)
    {
      point.x = nogo_boundary[i].points[j].x;
      point.y = nogo_boundary[i].points[j].y;
      point.z = nogo_boundary[i].points[j].z;
      pd_.nogo_boundary_marker_->marker_.points.push_back(point);
      point.x = nogo_boundary[i].points[j + 1].x;
      point.y = nogo_boundary[i].points[j + 1].y;
      point.z = nogo_boundary[i].points[j + 1].z;
      pd_.nogo_boundary_marker_->marker_.points.push_back(point);
    }
    point.x = nogo_boundary[i].points.back().x;
    point.y = nogo_boundary[i].points.back().y;
    point.z = nogo_boundary[i].points.back().z;
    pd_.nogo_boundary_marker_->marker_.points.push_back(point);
    point.x = nogo_boundary[i].points.front().x;
    point.y = nogo_boundary[i].points.front().y;
    point.z = nogo_boundary[i].points.front().z;
    pd_.nogo_boundary_marker_->marker_.points.push_back(point);
  }
  pd_.nogo_boundary_marker_->Publish();
}

void SensorCoveragePlanner3D::MobilityBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg)
{
  if (polygon_msg->polygon.points.empty())
  {
    return;
  }
  double polygon_id = polygon_msg->polygon.points[0].z;
  int polygon_point_size = polygon_msg->polygon.points.size();
  std::vector<geometry_msgs::Polygon> mobility_boundary;
  geometry_msgs::Polygon polygon;
  for (int i = 0; i < polygon_point_size; i++)
  {
    if (polygon_msg->polygon.points[i].z == polygon_id)
    {
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    }
    else
    {
      mobility_boundary.push_back(polygon);
      polygon.points.clear();
      polygon_id = polygon_msg->polygon.points[i].z;
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    }
  }
  mobility_boundary.push_back(polygon);
  pd_.grid_world_->UpdateMobilityBoundary(mobility_boundary);
  pd_.visualizer_->PlotMobilityBoundary(mobility_boundary);
}

void SensorCoveragePlanner3D::VirtualObstacleCallback(
    const sensor_msgs::PointCloud2ConstPtr& virtual_obstacle_cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr virtual_obstacle_cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*virtual_obstacle_cloud_msg, *virtual_obstacle_cloud_tmp);
  if (virtual_obstacle_cloud_tmp->points.empty())
  {
    return;
  }

  pd_.virtual_obstacle_cloud_->cloud_->clear();
  for (const auto& point : virtual_obstacle_cloud_tmp->points)
  {
    pcl::PointXYZI obstacle_point;
    obstacle_point.x = point.x;
    obstacle_point.y = point.y;
    obstacle_point.z = point.z;
    obstacle_point.intensity = 0;
    pd_.virtual_obstacle_cloud_->cloud_->points.push_back(obstacle_point);
  }
  pd_.virtual_obstacle_cloud_->Publish();
}

void SensorCoveragePlanner3D::CoveredPointCallback(const geometry_msgs::PointStamped::ConstPtr& covered_point_msg)
{
  int cell_ind =
      pd_.grid_world_->GetCellInd(covered_point_msg->point.x, covered_point_msg->point.y, covered_point_msg->point.z);
  if (pd_.grid_world_->IndInBound(cell_ind))
  {
    pd_.grid_world_->SetCellStatus(cell_ind, grid_world_ns::CellStatus::COVERED);
  }
}

void SensorCoveragePlanner3D::UnexploredPointCallback(const geometry_msgs::PointStamped::ConstPtr& unexplored_point_msg)
{
  int cell_ind = pd_.grid_world_->GetCellInd(unexplored_point_msg->point.x, unexplored_point_msg->point.y,
                                             unexplored_point_msg->point.z);
  if (pd_.grid_world_->IndInBound(cell_ind))
  {
    pd_.grid_world_->SetCellStatus(cell_ind, grid_world_ns::CellStatus::EXPLORING);
  }
}

void SensorCoveragePlanner3D::PlannerAggressiveCallback(const std_msgs::Bool::ConstPtr& start_msg)
{
  pd_.viewpoint_manager_->SetkLineOfSightStopAtNearestObstacle(!(start_msg->data));
}

void SensorCoveragePlanner3D::PauseCallback(const std_msgs::Bool::ConstPtr& pause_msg)
{
  pause_ = pause_msg->data;
}

void SensorCoveragePlanner3D::NavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& nav_goal_msg)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geo_quat = nav_goal_msg->pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);
  if (yaw > 0)
  {
    follow_manual_waypoint_ = true;
    pd_.manual_waypoint_.x = nav_goal_msg->pose.position.x;
    pd_.manual_waypoint_.y = nav_goal_msg->pose.position.y;
    pd_.manual_waypoint_.z = pd_.robot_position_.z;
  }
  else
  {
    follow_manual_waypoint_ = false;
  }
}

void SensorCoveragePlanner3D::InitialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial_pose_msg)
{
  pause_ = !pause_;
}

void SensorCoveragePlanner3D::FreePathsCallback(const sensor_msgs::PointCloud2ConstPtr& free_paths_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr free_paths_tmp(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*free_paths_msg, *free_paths_tmp);
  if (free_paths_tmp->points.empty())
  {
    free_paths_empty_count_++;
    if (free_paths_empty_count_ >= 2)
    {
      std_msgs::Float32 map_clearing_msg;
      map_clearing_msg.data = 8.0;
      map_clearing_publisher_.publish(map_clearing_msg);
      free_paths_empty_count_ = 0;
      ROS_WARN_STREAM("Clearing terrain map");
    }
  }
}

void SensorCoveragePlanner3D::SendInitialWaypoint()
{
  // send waypoint ahead
  double lx = 12.0;
  double ly = 0.0;
  double dx = cos(pd_.robot_yaw_) * lx - sin(pd_.robot_yaw_) * ly;
  double dy = sin(pd_.robot_yaw_) * lx + cos(pd_.robot_yaw_) * ly;

  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();
  waypoint.point.x = pd_.robot_position_.x + dx;
  waypoint.point.y = pd_.robot_position_.y + dy;
  waypoint.point.z = pd_.robot_position_.z;
  waypoint_pub_.publish(waypoint);
}

void SensorCoveragePlanner3D::UpdateKeyposeGraph()
{
  misc_utils_ns::Timer update_keypose_graph_timer("update keypose graph");
  update_keypose_graph_timer.Start();

  if (misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
          pd_.last_added_keypose_position_, pd_.robot_position_) > pp_.kAddKeyposeNodeMinDist)
  {
    pd_.last_added_keypose_position_ = pd_.robot_position_;
    pd_.cur_keypose_node_ind_ =
        pd_.keypose_graph_->AddKeyposeNode(pd_.robot_position_, *(pd_.planning_env_), pd_.viewpoint_manager_);
  }

  pd_.keypose_graph_->CheckLocalCollision(pd_.robot_position_, pd_.viewpoint_manager_);
  pd_.keypose_graph_->CheckConnectivity(pd_.robot_position_);

  pd_.keypose_graph_vis_cloud_->cloud_->clear();
  pd_.keypose_graph_->GetVisualizationCloud(pd_.keypose_graph_vis_cloud_->cloud_);
  pd_.keypose_graph_vis_cloud_->Publish();

  pd_.keypose_graph_->GetMarker(pd_.keypose_graph_node_marker_->marker_, pd_.keypose_graph_edge_marker_->marker_);
  pd_.keypose_graph_edge_marker_->Publish();

  update_keypose_graph_timer.Stop(false);
}

int SensorCoveragePlanner3D::UpdateViewPoints()
{
  misc_utils_ns::Timer viewpoint_manager_update_timer("update viewpoint manager");
  viewpoint_manager_update_timer.Start();
  if (pp_.kUseTerrainHeight)
  {
    pd_.viewpoint_manager_->SetViewPointHeightWithTerrain(pd_.large_terrain_cloud_->cloud_);
  }

  pd_.collision_cloud_->cloud_ = pd_.planning_env_->GetCollisionCloud();
  pd_.viewpoint_manager_->CheckViewPointCollision(pd_.collision_cloud_->cloud_);
  pd_.viewpoint_manager_->CheckViewPointLineOfSight();
  pd_.viewpoint_manager_->CheckViewPointConnectivity();
  int viewpoint_candidate_count = pd_.viewpoint_manager_->GetViewPointCandidate();

  pd_.viewpoint_manager_->UpdateViewPointVisited(pd_.grid_world_);
  UpdateVisitedPositions();
  pd_.viewpoint_manager_->UpdateViewPointVisited(pd_.visited_positions_);

  // For visualization
  pd_.collision_cloud_->Publish();
  pd_.viewpoint_manager_->GetCollisionViewPointVisCloud(pd_.viewpoint_in_collision_cloud_->cloud_);
  pd_.viewpoint_in_collision_cloud_->Publish();

  viewpoint_manager_update_timer.Stop(false);
  return viewpoint_candidate_count;
}

void SensorCoveragePlanner3D::UpdateViewPointCoverage()
{
  // Update viewpoint coverage
  misc_utils_ns::Timer update_coverage_timer("update viewpoint coverage");
  update_coverage_timer.Start();
  pd_.viewpoint_manager_->UpdateViewPointCoverage<PlannerCloudPointType>(pd_.planning_env_->GetDiffCloud());
  pd_.viewpoint_manager_->UpdateRolledOverViewPointCoverage<PlannerCloudPointType>(
      pd_.planning_env_->GetStackedCloud());
  // Update robot coverage
  pd_.robot_viewpoint_.ResetCoverage();
  geometry_msgs::Pose robot_pose;
  robot_pose.position = pd_.robot_position_;
  pd_.robot_viewpoint_.setPose(robot_pose);
  UpdateRobotViewPointCoverage();
  update_coverage_timer.Stop(false);
}

void SensorCoveragePlanner3D::UpdateRobotViewPointCoverage()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pd_.planning_env_->GetCollisionCloud();
  for (const auto& point : cloud->points)
  {
    if (pd_.viewpoint_manager_->InFOVAndRange(
            Eigen::Vector3d(point.x, point.y, point.z),
            Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z)))
    {
      pd_.robot_viewpoint_.UpdateCoverage<pcl::PointXYZI>(point);
    }
  }
}

void SensorCoveragePlanner3D::UpdateCoveredAreas(int& uncovered_point_num, int& uncovered_frontier_point_num)
{
  // Update covered area
  misc_utils_ns::Timer update_coverage_area_timer("update covered area");
  update_coverage_area_timer.Start();
  pd_.planning_env_->UpdateCoveredArea(pd_.robot_viewpoint_, pd_.viewpoint_manager_);
  update_coverage_area_timer.Stop(false);
  misc_utils_ns::Timer get_uncovered_area_timer("get uncovered area");
  get_uncovered_area_timer.Start();
  pd_.planning_env_->GetUncoveredArea(pd_.viewpoint_manager_, uncovered_point_num, uncovered_frontier_point_num);
  get_uncovered_area_timer.Stop(false);
  pd_.planning_env_->PublishUncoveredCloud();
  pd_.planning_env_->PublishUncoveredFrontierCloud();
}

void SensorCoveragePlanner3D::UpdateVisitedPositions()
{
  Eigen::Vector3d robot_current_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  bool existing = false;
  for (int i = 0; i < pd_.visited_positions_.size(); i++)
  {
    // TODO: parameterize this
    if ((robot_current_position - pd_.visited_positions_[i]).norm() < 1)
    {
      existing = true;
      break;
    }
  }
  if (!existing)
  {
    pd_.visited_positions_.push_back(robot_current_position);
  }
}

void SensorCoveragePlanner3D::UpdateGlobalRepresentation()
{
  pd_.local_coverage_planner_->SetRobotPosition(
      Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z));
  bool viewpoint_rollover = pd_.viewpoint_manager_->UpdateRobotPosition(
      Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z));
  if (!pd_.grid_world_->Initialized() || viewpoint_rollover)
  {
    pd_.grid_world_->UpdateNeighborCells(pd_.robot_position_);
  }

  pd_.planning_env_->UpdateRobotPosition(pd_.robot_position_);
  pd_.planning_env_->GetVisualizationPointCloud(pd_.point_cloud_manager_neighbor_cloud_->cloud_);
  pd_.point_cloud_manager_neighbor_cloud_->Publish();

  // DEBUG
  Eigen::Vector3d pointcloud_manager_neighbor_cells_origin =
      pd_.planning_env_->GetPointCloudManagerNeighborCellsOrigin();
  geometry_msgs::PointStamped pointcloud_manager_neighbor_cells_origin_point;
  pointcloud_manager_neighbor_cells_origin_point.header.frame_id = "map";
  pointcloud_manager_neighbor_cells_origin_point.header.stamp = ros::Time::now();
  pointcloud_manager_neighbor_cells_origin_point.point.x = pointcloud_manager_neighbor_cells_origin.x();
  pointcloud_manager_neighbor_cells_origin_point.point.y = pointcloud_manager_neighbor_cells_origin.y();
  pointcloud_manager_neighbor_cells_origin_point.point.z = pointcloud_manager_neighbor_cells_origin.z();
  pointcloud_manager_neighbor_cells_origin_pub_.publish(pointcloud_manager_neighbor_cells_origin_point);

  if (exploration_finished_ && pp_.kNoExplorationReturnHome)
  {
    pd_.planning_env_->SetUseFrontier(false);
  }
  else
  {
    pd_.planning_env_->SetUseFrontier(true);
  }

  pd_.planning_env_->UpdateKeyposeCloud<PlannerCloudPointType>(pd_.keypose_cloud_->cloud_);
  if (pp_.kCheckTerrainCollision)
  {
    pd_.planning_env_->AddToCollisionCloud(pd_.terrain_collision_cloud_->cloud_);
    pd_.planning_env_->AddToCollisionCloud(pd_.terrain_ext_collision_cloud_->cloud_);
  }
  if (pp_.kCheckVirtualObstacleCollision)
  {
    pd_.planning_env_->AddToCollisionCloud(pd_.virtual_obstacle_cloud_->cloud_);
  }

  int closest_node_ind = pd_.keypose_graph_->GetClosestNodeInd(pd_.robot_position_);
  geometry_msgs::Point closest_node_position = pd_.keypose_graph_->GetClosestNodePosition(pd_.robot_position_);
  pd_.grid_world_->SetCurKeyposeGraphNodeInd(closest_node_ind);
  pd_.grid_world_->SetCurKeyposeGraphNodePosition(closest_node_position);

  pd_.grid_world_->UpdateRobotPosition(pd_.robot_position_);
  if (!pd_.grid_world_->HomeSet())
  {
    pd_.grid_world_->SetHomePosition(pd_.initial_position_);
  }
}

void SensorCoveragePlanner3D::GlobalPlanning(std::vector<int>& global_cell_tsp_order,
                                             exploration_path_ns::ExplorationPath& global_path)
{
  misc_utils_ns::Timer global_tsp_timer("Global planning");
  global_tsp_timer.Start();

  pd_.multi_robot_exploration_info_manager_->UpdateRobotPosition(pd_.last_robot_position_);
  pd_.multi_robot_exploration_info_manager_->GetRobots(pd_.all_robots_);
  pd_.grid_world_->UpdateNeighboringCellPriority(pd_.all_robots_);

  pd_.grid_world_->UpdateLocalCellStatus(pd_.viewpoint_manager_);
  pd_.multi_robot_exploration_info_manager_->UpdateGlobalKnowledge(pd_.grid_world_);

  pd_.grid_world_->UpdateCellKeyposeGraphNodes(pd_.keypose_graph_);
  pd_.grid_world_->UpdateKeyposeGraphAndRoadmap(pd_.viewpoint_manager_, pd_.keypose_graph_, relocation_);
  pd_.grid_world_->UpdateGlobalCellStatus(pd_.keypose_graph_);

  pd_.viewpoint_manager_->UpdateCandidateViewPointCellStatus(pd_.grid_world_);

  misc_utils_ns::Timer vrp_timer("Global planning VRP");
  vrp_timer.Start();
  exploration_path_ns::ExplorationPath vrp_exploration_path = pd_.grid_world_->SolveGlobalVRP(
      pd_.viewpoint_manager_, pd_.all_robots_, global_cell_tsp_order, pd_.keypose_graph_);
  vrp_timer.Stop(false);

  global_path = vrp_exploration_path;

  vrp_exploration_path.GetVisualizationCloud(pd_.global_path_cloud_->cloud_);
  pd_.global_path_cloud_->Publish();

  global_tsp_timer.Stop(false);
  global_planning_runtime_ = global_tsp_timer.GetDuration("ms");

  std::vector<bool> finished_exploring;
  others_finished_exploring_ = OthersFinishedExploring(pd_.all_robots_, finished_exploring);

  // pd_.grid_world_->SaveGlobalPlanningStatusToFile(pd_.all_robots_);
  // SaveKeyposeGraphToFile();
}

bool SensorCoveragePlanner3D::OthersFinishedExploring(const std::vector<tare::Robot>& robots,
                                                      std::vector<bool>& finished_exploring)
{
  finished_exploring.resize(robots.size(), false);
  bool others_finished = true;
  for (int robot_id = 0; robot_id < robots.size(); robot_id++)
  {
    MY_ASSERT(robot_id == robots[robot_id].id_);
    if (robot_id == pd_.multi_robot_exploration_info_manager_->GetSelfID())
    {
      continue;
    }
    if (robots[robot_id].vrp_plan_.local_exploring_cell_ids_.empty())
    {
      if (robots[robot_id].vrp_plan_.no_comms_ordered_cell_ids_.empty())
      {
        finished_exploring[robot_id] = true;
      }
      else
      {
        if (robots[robot_id].vrp_plan_.no_comms_ordered_cell_ids_[robot_id].empty())
        {
          finished_exploring[robot_id] = true;
        }
      }
    }
    others_finished = others_finished && finished_exploring[robot_id];
  }

  return others_finished;
}

void SensorCoveragePlanner3D::PublishGlobalPlanningVisualization(
    const exploration_path_ns::ExplorationPath& global_path, const exploration_path_ns::ExplorationPath& local_path)
{
  nav_msgs::Path global_path_full = global_path.GetPath();
  global_path_full.header.frame_id = "map";
  global_path_full.header.stamp = ros::Time::now();
  global_path_full_publisher_.publish(global_path_full);
  // Get the part that connects with the local path

  int start_index = 0;
  for (int i = 0; i < global_path.nodes_.size(); i++)
  {
    exploration_path_ns::NodeType node_type = global_path.nodes_[i].type_;
    if (node_type == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        node_type == exploration_path_ns::NodeType::HOME)
    {
      break;
    }
    else if (node_type == exploration_path_ns::NodeType::GLOBAL_VIA_POINT &&
             !pd_.viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
    {
      break;
    }
    else if (node_type == exploration_path_ns::NodeType::GLOBAL_ROADMAP)
    {
      int cell_id = global_path.nodes_[i].global_subspace_index_;
      if (!pd_.grid_world_->IsNeighbor(cell_id) || pd_.grid_world_->GetCellViewPointIndices(cell_id).empty())
      {
        break;
      }
    }

    start_index = i;
  }

  int end_index = global_path.nodes_.size() - 1;
  for (int i = global_path.nodes_.size() - 1; i >= 0; i--)
  {
    exploration_path_ns::NodeType node_type = global_path.nodes_[i].type_;
    if (node_type == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        node_type == exploration_path_ns::NodeType::HOME)
    {
      break;
    }
    else if (node_type == exploration_path_ns::NodeType::GLOBAL_VIA_POINT &&
             !pd_.viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
    {
      break;
    }
    else if (node_type == exploration_path_ns::NodeType::GLOBAL_ROADMAP)
    {
      int cell_id = global_path.nodes_[i].global_subspace_index_;
      if (!pd_.grid_world_->IsNeighbor(cell_id) || pd_.grid_world_->GetCellViewPointIndices(cell_id).empty())
      {
        break;
      }
    }

    end_index = i;
  }

  nav_msgs::Path global_path_trim;
  if (local_path.nodes_.size() >= 2)
  {
    geometry_msgs::PoseStamped first_pose;
    first_pose.pose.position.x = local_path.nodes_.front().position_.x();
    first_pose.pose.position.y = local_path.nodes_.front().position_.y();
    first_pose.pose.position.z = local_path.nodes_.front().position_.z();
    global_path_trim.poses.push_back(first_pose);
  }

  for (int i = start_index; i <= end_index; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = global_path.nodes_[i].position_.x();
    pose.pose.position.y = global_path.nodes_[i].position_.y();
    pose.pose.position.z = global_path.nodes_[i].position_.z();
    global_path_trim.poses.push_back(pose);
  }
  if (local_path.nodes_.size() >= 2)
  {
    geometry_msgs::PoseStamped last_pose;
    last_pose.pose.position.x = local_path.nodes_.back().position_.x();
    last_pose.pose.position.y = local_path.nodes_.back().position_.y();
    last_pose.pose.position.z = local_path.nodes_.back().position_.z();
    global_path_trim.poses.push_back(last_pose);
  }
  global_path_trim.header.frame_id = "map";
  global_path_trim.header.stamp = ros::Time::now();
  global_path_publisher_.publish(global_path_trim);

  pd_.grid_world_->GetVisualizationCloud(pd_.grid_world_vis_cloud_->cloud_);
  pd_.grid_world_vis_cloud_->Publish();
  pd_.grid_world_->GetLowExploringPriorityCellsVisualizationCloud(pd_.low_priority_exploring_cell_vis_cloud_->cloud_);
  pd_.low_priority_exploring_cell_vis_cloud_->Publish();

  pd_.grid_world_->GetMarker(pd_.grid_world_marker_->marker_);
  pd_.grid_world_marker_->Publish();

  pd_.grid_world_->GetRoadmapMarker(pd_.global_roadmap_node_marker_->marker_, pd_.global_roadmap_edge_marker_->marker_);
  pd_.global_roadmap_node_marker_->Publish();
  pd_.global_roadmap_edge_marker_->Publish();

  nav_msgs::Path full_path = pd_.exploration_path_.GetPath();
  full_path.header.frame_id = "map";
  full_path.header.stamp = ros::Time::now();
  // exploration_path_publisher_.publish(full_path);
  pd_.exploration_path_.GetVisualizationCloud(pd_.exploration_path_cloud_->cloud_);
  pd_.exploration_path_cloud_->Publish();
  // pd_.planning_env_->PublishStackedCloud();
}

void SensorCoveragePlanner3D::LocalPlanning(int uncovered_point_num, int uncovered_frontier_point_num,
                                            const exploration_path_ns::ExplorationPath& global_path,
                                            exploration_path_ns::ExplorationPath& local_path)
{
  misc_utils_ns::Timer local_tsp_timer("Local planning");
  local_tsp_timer.Start();
  if (lookahead_point_update_)
  {
    pd_.local_coverage_planner_->SetLookAheadPoint(pd_.lookahead_point_);
  }
  local_path = pd_.local_coverage_planner_->SolveLocalCoverageProblem(global_path, uncovered_point_num,
                                                                      uncovered_frontier_point_num, pd_.grid_world_);
  local_tsp_timer.Stop(false);
}

void SensorCoveragePlanner3D::LocalPlanningToFollowGlobalPath(const exploration_path_ns::ExplorationPath& global_path,
                                                              exploration_path_ns::ExplorationPath& local_path)
{
  if (lookahead_point_update_)
  {
    pd_.local_coverage_planner_->SetLookAheadPoint(pd_.lookahead_point_);
  }
  local_path = pd_.local_coverage_planner_->FollowGlobalPath(global_path, pd_.grid_world_);
}

bool SensorCoveragePlanner3D::GetConvoyGlobalPath(exploration_path_ns::ExplorationPath& global_convoy_path)
{
  global_convoy_path = pd_.grid_world_->PlanGlobalConvoy(pd_.all_robots_, pd_.keypose_graph_);
  return !global_convoy_path.nodes_.empty();
}

void SensorCoveragePlanner3D::PublishLocalPlanningVisualization(const exploration_path_ns::ExplorationPath& local_path)
{
  pd_.viewpoint_manager_->GetVisualizationCloud(pd_.viewpoint_vis_cloud_->cloud_);
  pd_.viewpoint_vis_cloud_->Publish();
  pd_.lookahead_point_cloud_->Publish();
  nav_msgs::Path local_tsp_path = local_path.GetPath();
  local_tsp_path.header.frame_id = "map";
  local_tsp_path.header.stamp = ros::Time::now();
  local_tsp_path_publisher_.publish(local_tsp_path);
  pd_.local_coverage_planner_->GetSelectedViewPointVisCloud(pd_.selected_viewpoint_vis_cloud_->cloud_);
  pd_.selected_viewpoint_vis_cloud_->Publish();

  // Visualize local planning horizon box
}

void SensorCoveragePlanner3D::PublishCommsRelayPath(const exploration_path_ns::ExplorationPath& comms_relay_global_path,
                                                    const exploration_path_ns::ExplorationPath& comms_relay_local_path)
{
  nav_msgs::Path global_comms_relay_nav_path = comms_relay_global_path.GetPath();
  global_comms_relay_nav_path.header.frame_id = "map";
  global_comms_relay_nav_path.header.stamp = ros::Time::now();
  global_comms_relay_nav_path_publisher_.publish(global_comms_relay_nav_path);

  nav_msgs::Path local_comms_relay_nav_path = comms_relay_local_path.GetPath();
  local_comms_relay_nav_path.header.frame_id = "map";
  local_comms_relay_nav_path.header.stamp = ros::Time::now();
  local_comms_relay_nav_path_publisher_.publish(local_comms_relay_nav_path);
}

exploration_path_ns::ExplorationPath SensorCoveragePlanner3D::ConcatenateGlobalLocalPath(
    const exploration_path_ns::ExplorationPath& global_path, const exploration_path_ns::ExplorationPath& local_path)
{
  exploration_path_ns::ExplorationPath full_path;
  if (exploration_finished_ && near_home_ && pp_.kRushHome)
  {
    exploration_path_ns::Node node;
    node.position_.x() = pd_.robot_position_.x;
    node.position_.y() = pd_.robot_position_.y;
    node.position_.z() = pd_.robot_position_.z;
    node.type_ = exploration_path_ns::NodeType::ROBOT;
    full_path.nodes_.push_back(node);
    node.position_ = pd_.initial_position_;
    node.type_ = exploration_path_ns::NodeType::HOME;
    full_path.nodes_.push_back(node);
    return full_path;
  }

  double global_path_length = global_path.GetLength();
  double local_path_length = local_path.GetLength();
  if (global_path_length < 3 && local_path_length < 5)
  {
    return full_path;
  }
  else
  {
    full_path = local_path;
    if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
        local_path.nodes_.back().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START)
    {
      full_path.Reverse();
    }
    else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START &&
             local_path.nodes_.back() == local_path.nodes_.front())
    {
      full_path.nodes_.back().type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
    }
    else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
             local_path.nodes_.back() == local_path.nodes_.front())
    {
      full_path.nodes_.front().type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
    }
  }

  return full_path;
}

void SensorCoveragePlanner3D::GetNextGlobalSubspace(const exploration_path_ns::ExplorationPath& global_path,
                                                    int next_subspace_index, double distance_from_start)
{
  next_subspace_index = -1;
  distance_from_start = 0.0;
  for (int i = 1; i < global_path.nodes_.size(); i++)
  {
    distance_from_start += (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_).norm();
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT)
    {
      next_subspace_index = global_path.nodes_[i].global_subspace_index_;
      break;
    }
  }
}

bool SensorCoveragePlanner3D::GetLookAheadPoint(const exploration_path_ns::ExplorationPath& local_path,
                                                const exploration_path_ns::ExplorationPath& global_path,
                                                Eigen::Vector3d& lookahead_point, bool& follow_local_path_from_start)
{
  Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  lookahead_point = robot_position;

  exploration_path_ns::ExplorationPath global_path_reverse = global_path;
  global_path_reverse.Reverse();

  exploration_path_ns::ExplorationPath local_path_reverse = local_path;
  local_path_reverse.Reverse();

  // Determine which direction to follow on the global path
  int forward_global_subspace_index = -1;
  double dist_from_start = 0.0;
  GetNextGlobalSubspace(global_path, forward_global_subspace_index, dist_from_start);

  int backward_global_subspace_index = -1;
  double dist_from_end = 0.0;
  GetNextGlobalSubspace(global_path_reverse, backward_global_subspace_index, dist_from_end);

  next_global_subspace_index_ =
      dist_from_start <= dist_from_end ? forward_global_subspace_index : backward_global_subspace_index;

  bool local_path_too_short = true;
  for (int i = 0; i < local_path.nodes_.size(); i++)
  {
    double dist_to_robot = (robot_position - local_path.nodes_[i].position_).norm();
    if (dist_to_robot > pp_.kLookAheadDistance / 5)
    {
      local_path_too_short = false;
      break;
    }
  }

  if (local_path.GetNodeNum() < 1 || local_path_too_short)
  {
    // Follow the global path directly
    exploration_path_ns::ExplorationPath subsampled_global_path = global_path.Subsample(0.5);
    exploration_path_ns::ExplorationPath subsampled_global_path_reverse = global_path_reverse.Subsample(0.5);

    if (!subsampled_global_path.nodes_.empty() && dist_from_start < dist_from_end)
    {
      lookahead_point = subsampled_global_path.GetNodeDistanceFromBegin(1.5).position_;
    }
    else if (!subsampled_global_path_reverse.nodes_.empty() && dist_from_start >= dist_from_end)
    {
      lookahead_point = subsampled_global_path_reverse.GetNodeDistanceFromBegin(1.5).position_;
    }
    // ROS_INFO_STREAM("follow global path directly");
    return false;
  }

  int lookahead_i = 0;
  int robot_i = 0;
  bool has_lookahead_node = local_path.HasNodeType(exploration_path_ns::NodeType::LOOKAHEAD_POINT, lookahead_i);
  bool has_robot_node = local_path.HasNodeType(exploration_path_ns::NodeType::ROBOT, robot_i);

  int forward_viewpoint_count = 0;
  int backward_viewpoint_count = 0;

  bool local_loop = false;
  if (local_path.nodes_.front() == local_path.nodes_.back() &&
      local_path.nodes_.front().type_ == exploration_path_ns::NodeType::ROBOT)
  {
    local_loop = true;
  }

  if (local_loop)
  {
    robot_i = 0;
  }
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++)
  {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT)
    {
      forward_viewpoint_count++;
    }
  }
  if (local_loop)
  {
    robot_i = local_path.nodes_.size() - 1;
  }
  for (int i = robot_i - 1; i >= 0; i--)
  {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT)
    {
      backward_viewpoint_count++;
    }
  }

  Eigen::Vector3d forward_lookahead_point = robot_position;
  Eigen::Vector3d backward_lookahead_point = robot_position;

  bool has_forward = false;
  bool has_backward = false;

  if (local_loop)
  {
    robot_i = 0;
  }
  bool forward_lookahead_point_in_los = true;
  bool backward_lookahead_point_in_los = true;
  double length_from_robot = 0.0;
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++)
  {
    int cell_id = pd_.grid_world_->GetCellInd(local_path.nodes_[i].position_.x(), local_path.nodes_[i].position_.y(),
                                              local_path.nodes_[i].position_.z());
    if ((forward_viewpoint_count > 0 || backward_viewpoint_count > 0) &&
        !pd_.grid_world_->IsCellHasExploringPriority(cell_id))
    {
      continue;
    }

    bool in_line_of_sight = true;
    if (i < local_path.GetNodeNum() - 1)
    {
      in_line_of_sight = pd_.viewpoint_manager_->InCurrentFrameLineOfSight(local_path.nodes_[i + 1].position_);
    }

    length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i - 1].position_).norm();
    if ((length_from_robot > pp_.kLookAheadDistance || (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END ||
         i == local_path.GetNodeNum() - 1))

    {
      if (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight)
      {
        forward_lookahead_point_in_los = false;
      }
      forward_lookahead_point = local_path.nodes_[i].position_;
      has_forward = true;
      break;
    }
  }
  if (local_loop)
  {
    robot_i = local_path.nodes_.size() - 1;
  }
  length_from_robot = 0.0;
  for (int i = robot_i - 1; i >= 0; i--)
  {
    int cell_id = pd_.grid_world_->GetCellInd(local_path.nodes_[i].position_.x(), local_path.nodes_[i].position_.y(),
                                              local_path.nodes_[i].position_.z());
    if ((forward_viewpoint_count > 0 || backward_viewpoint_count > 0) &&
        !pd_.grid_world_->IsCellHasExploringPriority(cell_id))
    {
      continue;
    }

    bool in_line_of_sight = true;
    if (i > 0)
    {
      in_line_of_sight = pd_.viewpoint_manager_->InCurrentFrameLineOfSight(local_path.nodes_[i - 1].position_);
    }

    length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i + 1].position_).norm();
    if ((length_from_robot > pp_.kLookAheadDistance || (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END || i == 0))

    {
      if (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight)
      {
        backward_lookahead_point_in_los = false;
      }
      backward_lookahead_point = local_path.nodes_[i].position_;
      has_backward = true;
      break;
    }
  }

  double dx = pd_.lookahead_point_direction_.x();
  double dy = pd_.lookahead_point_direction_.y();

  double forward_angle_score = -2;
  double backward_angle_score = -2;
  double lookahead_angle_score = -2;

  double dist_robot_to_lookahead = 0.0;
  if (has_forward)
  {
    Eigen::Vector3d forward_diff = forward_lookahead_point - robot_position;
    forward_diff.z() = 0.0;
    forward_diff = forward_diff.normalized();
    forward_angle_score = dx * forward_diff.x() + dy * forward_diff.y();
  }
  if (has_backward)
  {
    Eigen::Vector3d backward_diff = backward_lookahead_point - robot_position;
    backward_diff.z() = 0.0;
    backward_diff = backward_diff.normalized();
    backward_angle_score = dx * backward_diff.x() + dy * backward_diff.y();
  }
  if (has_lookahead_node)
  {
    Eigen::Vector3d prev_lookahead_point = local_path.nodes_[lookahead_i].position_;
    dist_robot_to_lookahead = (robot_position - prev_lookahead_point).norm();
    Eigen::Vector3d diff = prev_lookahead_point - robot_position;
    diff.z() = 0.0;
    diff = diff.normalized();
    lookahead_angle_score = dx * diff.x() + dy * diff.y();
  }

  pd_.lookahead_point_cloud_->cloud_->clear();

  if (forward_viewpoint_count == 0 && backward_viewpoint_count == 0)
  {
    relocation_ = true;
  }
  else
  {
    relocation_ = false;
  }

  if (relocation_)
  {
    if (use_momentum_ && pp_.kUseMomentum)
    {
      if (forward_angle_score > backward_angle_score)
      {
        lookahead_point = forward_lookahead_point;
        follow_local_path_from_start = true;
      }
      else
      {
        lookahead_point = backward_lookahead_point;
        follow_local_path_from_start = false;
      }
    }
    else
    {
      // follow the shorter distance one
      if (dist_from_start < dist_from_end && local_path.nodes_.front().type_ != exploration_path_ns::NodeType::ROBOT)
      {
        lookahead_point = backward_lookahead_point;
        follow_local_path_from_start = false;
      }
      else if (dist_from_end < dist_from_start &&
               local_path.nodes_.back().type_ != exploration_path_ns::NodeType::ROBOT)
      {
        lookahead_point = forward_lookahead_point;
        follow_local_path_from_start = true;
      }
      else
      {
        lookahead_point =
            forward_angle_score > backward_angle_score ? forward_lookahead_point : backward_lookahead_point;
        follow_local_path_from_start = forward_angle_score > backward_angle_score ? true : false;
      }
    }
  }
  else
  {
    // Exploring
    if (forward_angle_score > backward_angle_score)
    {
      if (forward_viewpoint_count > 0)
      {
        lookahead_point = forward_lookahead_point;
        follow_local_path_from_start = true;
      }
      else
      {
        lookahead_point = backward_lookahead_point;
        follow_local_path_from_start = false;
      }
    }
    else
    {
      if (backward_viewpoint_count > 0)
      {
        lookahead_point = backward_lookahead_point;
        follow_local_path_from_start = false;
      }
      else
      {
        lookahead_point = forward_lookahead_point;
        follow_local_path_from_start = true;
      }
    }
  }

  if ((lookahead_point == forward_lookahead_point && !forward_lookahead_point_in_los) ||
      (lookahead_point == backward_lookahead_point && !backward_lookahead_point_in_los))
  {
    lookahead_point_in_line_of_sight_ = false;
  }
  else
  {
    lookahead_point_in_line_of_sight_ = true;
  }

  pd_.lookahead_point_direction_ = lookahead_point - robot_position;
  pd_.lookahead_point_direction_.z() = 0.0;
  pd_.lookahead_point_direction_.normalize();

  pcl::PointXYZI point;
  point.x = lookahead_point.x();
  point.y = lookahead_point.y();
  point.z = lookahead_point.z();
  point.intensity = 1.0;
  pd_.lookahead_point_cloud_->cloud_->points.push_back(point);
  if (has_lookahead_node)
  {
    point.x = local_path.nodes_[lookahead_i].position_.x();
    point.y = local_path.nodes_[lookahead_i].position_.y();
    point.z = local_path.nodes_[lookahead_i].position_.z();
    point.intensity = 0;
    pd_.lookahead_point_cloud_->cloud_->points.push_back(point);
  }
  return true;
}

void SensorCoveragePlanner3D::PublishWaypoint()
{
  geometry_msgs::PointStamped waypoint;
  if (follow_manual_waypoint_)
  {
    waypoint.point = pd_.manual_waypoint_;
  }
  else if (pd_.grid_world_->WaitForOtherRobots())
  {
    waypoint.point = pd_.robot_position_;
  }
  else
  {
    if (exploration_finished_ && near_home_ && pp_.kRushHome)
    {
      waypoint.point.x = pd_.initial_position_.x();
      waypoint.point.y = pd_.initial_position_.y();
      waypoint.point.z = pd_.initial_position_.z();
    }
    else
    {
      double dx = pd_.lookahead_point_.x() - pd_.robot_position_.x;
      double dy = pd_.lookahead_point_.y() - pd_.robot_position_.y;
      double r = sqrt(dx * dx + dy * dy);
      // TODO: tmp fix to check if waypoint is at robot's position
      double extend_dist =
          lookahead_point_in_line_of_sight_ ? pp_.kExtendWayPointDistanceBig : pp_.kExtendWayPointDistanceSmall;
      if (r > 0.1 && r < extend_dist && pp_.kExtendWayPoint)
      {
        dx = dx / r * extend_dist;
        dy = dy / r * extend_dist;
      }
      waypoint.point.x = dx + pd_.robot_position_.x;
      waypoint.point.y = dy + pd_.robot_position_.y;
      waypoint.point.z = pd_.lookahead_point_.z();
    }
  }

  misc_utils_ns::Publish<geometry_msgs::PointStamped>(waypoint_pub_, waypoint, kWorldFrameID);
}

void SensorCoveragePlanner3D::PublishRuntime()
{
  local_viewpoint_sampling_runtime_ = pd_.local_coverage_planner_->GetViewPointSamplingRuntime() / 1000;
  local_path_finding_runtime_ =
      (pd_.local_coverage_planner_->GetFindPathRuntime() + pd_.local_coverage_planner_->GetTSPRuntime()) / 1000;

  std_msgs::Int32MultiArray runtime_breakdown_msg;
  runtime_breakdown_msg.data.clear();
  runtime_breakdown_msg.data.push_back(update_representation_runtime_);
  runtime_breakdown_msg.data.push_back(local_viewpoint_sampling_runtime_);
  runtime_breakdown_msg.data.push_back(local_path_finding_runtime_);
  runtime_breakdown_msg.data.push_back(global_planning_runtime_);
  runtime_breakdown_msg.data.push_back(trajectory_optimization_runtime_);
  runtime_breakdown_msg.data.push_back(overall_runtime_);
  runtime_breakdown_pub_.publish(runtime_breakdown_msg);

  float runtime = 0;
  if (!exploration_finished_ && pp_.kNoExplorationReturnHome)
  {
    for (int i = 0; i < runtime_breakdown_msg.data.size() - 1; i++)
    {
      runtime += runtime_breakdown_msg.data[i];
    }
  }

  std_msgs::Float32 runtime_msg;
  runtime_msg.data = runtime / 1000.0;
  runtime_pub_.publish(runtime_msg);
}

double SensorCoveragePlanner3D::GetRobotToHomeDistance()
{
  Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  return (robot_position - pd_.initial_position_).norm();
}

void SensorCoveragePlanner3D::PublishExplorationState()
{
  std_msgs::Bool exploration_finished_msg;
  exploration_finished_msg.data = exploration_finished_;
  exploration_finish_pub_.publish(exploration_finished_msg);
  robot_exploration_finish_pub_.publish(exploration_finished_msg);
}

void SensorCoveragePlanner3D::PrintExplorationStatus(std::string status, bool clear_last_line)
{
  if (clear_last_line)
  {
    printf(cursup);
    printf(cursclean);
    printf(cursup);
    printf(cursclean);
  }
  std::cout << std::endl << "\033[1;32m" << status << "\033[0m" << std::endl;
}

void SensorCoveragePlanner3D::CountDirectionChange()
{
  Eigen::Vector3d current_moving_direction_ =
      Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z) -
      Eigen::Vector3d(pd_.last_robot_position_.x, pd_.last_robot_position_.y, pd_.last_robot_position_.z);

  if (current_moving_direction_.norm() > 0.5)
  {
    if (pd_.moving_direction_.dot(current_moving_direction_) < 0)
    {
      direction_change_count_++;
      direction_no_change_count_ = 0;
      if (direction_change_count_ > pp_.kDirectionChangeCounterThr)
      {
        if (!use_momentum_)
        {
          momentum_activation_count_++;
        }
        use_momentum_ = true;
      }
    }
    else
    {
      direction_no_change_count_++;
      if (direction_no_change_count_ > pp_.kDirectionNoChangeCounterThr)
      {
        direction_change_count_ = 0;
        use_momentum_ = false;
      }
    }
    pd_.moving_direction_ = current_moving_direction_;
  }

  std_msgs::Int32 momentum_activation_count_msg;
  momentum_activation_count_msg.data = momentum_activation_count_;
  momentum_activation_count_pub_.publish(momentum_activation_count_msg);
}

void SensorCoveragePlanner3D::PublishPlannerAggressiveness()
{
  std_msgs::Bool aggressive_msg;
  aggressive_msg.data = !(pd_.viewpoint_manager_->GetkLineOfSightStopAtNearestObstacle());
  planner_aggressiveness_pub_.publish(aggressive_msg);
}

void SensorCoveragePlanner3D::PublishExplorationInfo()
{
  tare_msgs::ExplorationInfo exploration_info;
  pd_.grid_world_->GetExplorationInfo(pd_.keypose_graph_, exploration_info);
  exploration_info.secs_from_start = static_cast<uint32_t>((ros::Time::now() - start_time_).toSec());
  exploration_info.robot_positions.clear();
  exploration_info.robot_update_secs.clear();
  exploration_info.robot_update_secs.push_back(pd_.multi_robot_exploration_info_manager_->GetInRangeRobotIDs());
  for (int i = 0; i < pd_.all_robots_.size(); i++)
  {
    if (i == pd_.multi_robot_exploration_info_manager_->GetSelfID())
    {
      exploration_info.robot_positions.push_back(pd_.robot_position_);
      exploration_info.robot_update_secs.push_back(exploration_info.secs_from_start);
    }
    else
    {
      exploration_info.robot_positions.push_back(pd_.all_robots_[i].in_comms_position_);
      exploration_info.robot_update_secs.push_back(pd_.all_robots_[i].secs_from_start_);
    }
  }
  exploration_info_pub_.publish(exploration_info);

  // Publish position
  geometry_msgs::PointStamped position_msg;
  position_msg.header.stamp = ros::Time::now();
  position_msg.header.frame_id = kWorldFrameID;
  position_msg.point = pd_.robot_position_;
  robot_position_pub_.publish(position_msg);

  int VRP_longest_route_length = pd_.grid_world_->GetGlobalVRPLongestPathLength();
  std_msgs::Int32 route_length_msg;
  route_length_msg.data = VRP_longest_route_length;
  vrp_longest_route_length_publisher_.publish(route_length_msg);
}

void SensorCoveragePlanner3D::execute(const ros::TimerEvent&)
{
  if (!pp_.kAutoStart && !start_exploration_)
  {
    ROS_INFO("Waiting for start signal");
    return;
  }

  if (pause_)
  {
    return;
  }
  Timer overall_processing_timer("overall processing");
  update_representation_runtime_ = 0;
  local_viewpoint_sampling_runtime_ = 0;
  local_path_finding_runtime_ = 0;
  global_planning_runtime_ = 0;
  trajectory_optimization_runtime_ = 0;
  overall_runtime_ = 0;

  if (!initialized_)
  {
    SendInitialWaypoint();
    return;
  }

  // ROS_INFO("-----Init-----");

  if (!first_initialization_)
  {
    first_initialization_ = true;
    start_time_ = ros::Time::now();
    global_direction_switch_time_ = ros::Time::now();
  }

  overall_processing_timer.Start();
  if (keypose_cloud_update_)
  {
    keypose_cloud_update_ = false;

    CountDirectionChange();

    misc_utils_ns::Timer update_representation_timer("update representation");
    update_representation_timer.Start();

    // Update grid world
    UpdateGlobalRepresentation();

    int viewpoint_candidate_count = UpdateViewPoints();
    if (viewpoint_candidate_count == 0)
    {
      ROS_WARN("Cannot get candidate viewpoints, skipping this round");
      return;
    }

    UpdateKeyposeGraph();

    int uncovered_point_num = 0;
    int uncovered_frontier_point_num = 0;
    if (!exploration_finished_ || !pp_.kNoExplorationReturnHome)
    {
      UpdateViewPointCoverage();
      UpdateCoveredAreas(uncovered_point_num, uncovered_frontier_point_num);
    }
    else
    {
      pd_.viewpoint_manager_->ResetViewPointCoverage();
    }

    update_representation_timer.Stop(false);
    update_representation_runtime_ += update_representation_timer.GetDuration("ms");

    pd_.multi_robot_exploration_info_manager_->UpdateCommsStatus(pd_.last_robot_position_);
    pd_.last_robot_position_ = pd_.robot_position_;

    if (relay_comms_)
    {
      comms_status_ = tare::CommsStatus::RELAY_COMMS;
    }
    else if (convoy_)
    {
      comms_status_ = tare::CommsStatus::CONVOY;
    }
    else if (pd_.multi_robot_exploration_info_manager_->IsInComms())
    {
      comms_status_ = tare::CommsStatus::IN_COMMS;
    }
    else
    {
      comms_status_ = tare::CommsStatus::LOST_COMMS;
    }
    pd_.visualizer_->PlotCommsRange(comms_status_, pd_.robot_position_,
                                    pd_.multi_robot_exploration_info_manager_->GetCommsRange());

    // Global TSP
    std::vector<int> global_cell_tsp_order;
    exploration_path_ns::ExplorationPath global_path;
    GlobalPlanning(global_cell_tsp_order, global_path);

    // Local TSP
    exploration_path_ns::ExplorationPath local_path;
    LocalPlanning(uncovered_point_num, uncovered_frontier_point_num, global_path, local_path);

    int viewpoint_count = local_path.GetNodeCount(exploration_path_ns::NodeType::LOCAL_VIEWPOINT);

    exploration_path_ns::ExplorationPath global_convoy_path;
    convoy_ = false;
    if (global_cell_tsp_order.size() <= 2 && viewpoint_count == 0)
    {
      convoy_ = GetConvoyGlobalPath(global_convoy_path);
    }

    relay_comms_ = pd_.grid_world_->RelayComms() || pd_.grid_world_->GoToRendezvous() ||
                   (pp_.kUseTimeBudget && pd_.grid_world_->TimeToGoHome());

    exploration_path_ns::ExplorationPath comms_relay_global_path;
    if (relay_comms_)
    {
      comms_relay_global_path = global_path;
    }

    if (!relay_comms_ && convoy_)
    {
      global_path = global_convoy_path;
    }

    exploration_path_ns::ExplorationPath no_coverage_local_path;
    if (relay_comms_ || convoy_)
    {
      LocalPlanningToFollowGlobalPath(global_path, no_coverage_local_path);
      local_path = no_coverage_local_path;
    }

    std_msgs::Bool relay_comms;
    relay_comms.data = relay_comms_;
    relay_comms_publisher_.publish(relay_comms);

    // Tmp fix
    pd_.grid_world_->UpdateLocalCellStatus(pd_.viewpoint_manager_);
    pd_.viewpoint_manager_->UpdateCandidateViewPointCellStatus(pd_.grid_world_);

    near_home_ = GetRobotToHomeDistance() < pp_.kRushHomeDist;
    at_home_ = GetRobotToHomeDistance() < pp_.kAtHomeDistThreshold;

    if (pd_.grid_world_->NoExploringCells() && pd_.local_coverage_planner_->IsLocalCoverageComplete() &&
        (ros::Time::now() - start_time_).toSec() > 300)
    {
      exploration_finished_ = true;
    }
    else
    {
      exploration_finished_ = false;
    }

    if (exploration_finished_ && at_home_ && !stopped_)
    {
      PrintExplorationStatus("Return home completed", false);
      stopped_ = true;
    }
    else if (pp_.kUseTimeBudget && pd_.grid_world_->TimeToGoHome() && at_home_)
    {
      pd_.grid_world_->ResetTimeBudget();
      stopped_ = false;
    }

    pd_.exploration_path_ = ConcatenateGlobalLocalPath(global_path, local_path);

    PublishExplorationState();

    bool follow_local_path_from_start = true;
    lookahead_point_update_ =
        GetLookAheadPoint(pd_.exploration_path_, global_path, pd_.lookahead_point_, follow_local_path_from_start);
    PublishWaypoint();

    overall_processing_timer.Stop(false);
    overall_runtime_ = overall_processing_timer.GetDuration("ms");

    pd_.visualizer_->GetGlobalSubspaceMarker(pd_.grid_world_, global_cell_tsp_order);
    pd_.visualizer_->GetExploringCellMarkers(pd_.grid_world_);
    pd_.visualizer_->GetVRPPathMarkers(pd_.grid_world_);
    pd_.visualizer_->PublishSyncedSubspaces(pd_.grid_world_);
    Eigen::Vector3d viewpoint_origin = pd_.viewpoint_manager_->GetOrigin();
    pd_.visualizer_->GetLocalPlanningHorizonMarker(viewpoint_origin.x(), viewpoint_origin.y(), pd_.robot_position_.z);
    pd_.visualizer_->PublishMarkers();
    pd_.visualizer_->VisualizeRobotPositions(pd_.all_robots_);
    pd_.visualizer_->PublishRendezvousPosition(pd_.grid_world_->GetRendezvousPosition());

    PublishLocalPlanningVisualization(local_path);
    PublishGlobalPlanningVisualization(global_path, local_path);
    PublishCommsRelayPath(comms_relay_global_path, no_coverage_local_path);
    PublishRuntime();

    // Multi-robot coordination
    PublishExplorationInfo();
  }
}

void SensorCoveragePlanner3D::SaveKeyposeGraphToFile()
{
  std::string common_path = ros::package::getPath("tare_planner") + "/log/";

  std::string keypose_graph_nodes_filename = common_path + "robot" + std::to_string(id_) + "_keypose_graph_nodes.txt";
  std::string keypose_graph_edges_filename = common_path + "robot" + std::to_string(id_) + "_keypose_graph_edges.txt";

  std::vector<keypose_graph_ns::KeyposeNode> keypose_graph_nodes;
  pd_.keypose_graph_->GetNodes(keypose_graph_nodes);
  misc_utils_ns::SaveVectorToFile<keypose_graph_ns::KeyposeNode>(keypose_graph_nodes, keypose_graph_nodes_filename);

  std::vector<std::vector<int>> keypose_graph_edges;
  pd_.keypose_graph_->GetEdges(keypose_graph_edges);
  misc_utils_ns::Save2DVectorToFile<int>(keypose_graph_edges, keypose_graph_edges_filename);
}
}  // namespace sensor_coverage_planner_3d_ns
