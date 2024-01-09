/**
 * @file sensor_coverage_planner_ground.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>
// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// PCL
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
// Third parties
#include <utils/pointcloud_utils.h>
#include <utils/misc_utils.h>
// Components
#include "keypose_graph/keypose_graph.h"
#include "planning_env/planning_env.h"
#include "viewpoint_manager/viewpoint_manager.h"
#include "grid_world/grid_world.h"
#include "exploration_path/exploration_path.h"
#include "local_coverage_planner/local_coverage_planner.h"
#include "tare_visualizer/tare_visualizer.h"
#include "rolling_occupancy_grid/rolling_occupancy_grid.h"
#include "coordination/multi_robot_exploration_manager.h"

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

namespace sensor_coverage_planner_3d_ns
{
const std::string kWorldFrameID = "map";
typedef pcl::PointXYZRGBNormal PlannerCloudPointType;
typedef pcl::PointCloud<PlannerCloudPointType> PlannerCloudType;
typedef misc_utils_ns::Timer Timer;

struct PlannerParameters
{
  // String
  std::string sub_start_exploration_topic_;
  std::string sub_keypose_topic_;
  std::string sub_state_estimation_topic_;
  std::string sub_registered_scan_topic_;
  std::string sub_terrain_map_topic_;
  std::string sub_terrain_map_ext_topic_;
  std::string sub_coverage_boundary_topic_;
  std::string sub_viewpoint_boundary_topic_;
  std::string sub_nogo_boundary_topic_;
  std::string sub_mobility_boundary_topic_;
  std::string sub_virtual_obstacle_topic_;
  std::string sub_covered_point_topic_;
  std::string sub_unexplored_point_topic_;

  std::string pub_exploration_finish_topic_;
  std::string pub_runtime_breakdown_topic_;
  std::string pub_runtime_topic_;
  std::string pub_waypoint_topic_;
  std::string pub_momentum_activation_count_topic_;
  std::string pub_comms_relay_topic_;

  // For coordination
  // std::string robot_name_;
  std::string pub_exploration_info_topic_;
  std::string pub_robot_position_topic_;

  // Bool
  bool kAutoStart;
  bool kRushHome;
  bool kUseTerrainHeight;
  bool kCheckTerrainCollision;
  bool kCheckVirtualObstacleCollision;
  bool kExtendWayPoint;
  bool kUseLineOfSightLookAheadPoint;
  bool kNoExplorationReturnHome;
  bool kUseMomentum;
  bool kUseTimeBudget;

  // Double
  double kKeyposeCloudDwzFilterLeafSize;
  double kRushHomeDist;
  double kAtHomeDistThreshold;
  double kTerrainCollisionThreshold;
  double kLookAheadDistance;
  double kExtendWayPointDistanceBig;
  double kExtendWayPointDistanceSmall;
  double kAddKeyposeNodeMinDist;

  // Int
  int kDirectionChangeCounterThr;
  int kDirectionNoChangeCounterThr;

  // // String
  // std::string kTestID;

  bool ReadParameters(ros::NodeHandle& nh);
};

struct PlannerData
{
  // PCL clouds TODO: keypose cloud does not need to be PlannerCloudPointType
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> keypose_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>> registered_scan_stack_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> registered_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> large_terrain_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> terrain_collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> terrain_ext_collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> virtual_obstacle_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> viewpoint_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> grid_world_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> low_priority_exploring_cell_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> selected_viewpoint_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> exploring_cell_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> exploration_path_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> global_path_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> lookahead_point_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> keypose_graph_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> viewpoint_in_collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> point_cloud_manager_neighbor_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> reordered_global_subspace_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> subsampled_global_path_cloud_;

  nav_msgs::Odometry keypose_;
  geometry_msgs::Point robot_position_;
  geometry_msgs::Point last_robot_position_;
  geometry_msgs::Point last_added_keypose_position_;
  geometry_msgs::Point manual_waypoint_;
  lidar_model_ns::LiDARModel robot_viewpoint_;
  exploration_path_ns::ExplorationPath exploration_path_;
  Eigen::Vector3d lookahead_point_;
  Eigen::Vector3d lookahead_point_direction_;
  Eigen::Vector3d moving_direction_;
  double robot_yaw_;
  bool moving_forward_;
  std::vector<Eigen::Vector3d> visited_positions_;
  int cur_keypose_node_ind_;
  Eigen::Vector3d initial_position_;
  std::vector<tare::Robot> all_robots_;

  // Use shared_pointer
  std::unique_ptr<keypose_graph_ns::KeyposeGraph> keypose_graph_;
  std::unique_ptr<planning_env_ns::PlanningEnv> planning_env_;
  std::shared_ptr<viewpoint_manager_ns::ViewPointManager> viewpoint_manager_;
  std::unique_ptr<local_coverage_planner_ns::LocalCoveragePlanner> local_coverage_planner_;
  std::unique_ptr<grid_world_ns::GridWorld> grid_world_;
  std::unique_ptr<tare_visualizer_ns::TAREVisualizer> visualizer_;
  std::unique_ptr<tare::MultiRobotExplorationManager> multi_robot_exploration_info_manager_;

  std::unique_ptr<misc_utils_ns::Marker> keypose_graph_node_marker_;
  std::unique_ptr<misc_utils_ns::Marker> keypose_graph_edge_marker_;
  std::unique_ptr<misc_utils_ns::Marker> nogo_boundary_marker_;
  std::unique_ptr<misc_utils_ns::Marker> grid_world_marker_;
  std::unique_ptr<misc_utils_ns::Marker> global_roadmap_node_marker_;
  std::unique_ptr<misc_utils_ns::Marker> global_roadmap_edge_marker_;

  void Initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
};

class SensorCoveragePlanner3D
{
public:
  explicit SensorCoveragePlanner3D(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
  bool initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
  void execute(const ros::TimerEvent&);
  ~SensorCoveragePlanner3D() = default;

private:
  bool keypose_cloud_update_;
  bool initialized_;
  bool first_initialization_;
  bool lookahead_point_update_;
  bool relocation_;
  bool start_exploration_;
  bool exploration_finished_;
  bool others_finished_exploring_;
  bool near_home_;
  bool at_home_;
  bool stopped_;
  bool test_point_update_;
  bool viewpoint_ind_update_;
  bool step_;
  bool use_momentum_;
  bool lookahead_point_in_line_of_sight_;
  bool pause_;
  bool follow_manual_waypoint_;
  bool convoy_;
  bool relay_comms_;
  int id_;
  tare::CommsStatus comms_status_;
  PlannerParameters pp_;
  PlannerData pd_;
  pointcloud_utils_ns::PointCloudDownsizer<pcl::PointXYZ> pointcloud_downsizer_;

  int update_representation_runtime_;
  int local_viewpoint_sampling_runtime_;
  int local_path_finding_runtime_;
  int global_planning_runtime_;
  int trajectory_optimization_runtime_;
  int overall_runtime_;
  int registered_cloud_count_;
  int keypose_count_;
  int direction_change_count_;
  int direction_no_change_count_;
  int momentum_activation_count_;
  int next_global_subspace_index_;
  int free_paths_empty_count_;

  ros::Time start_time_;
  ros::Time global_direction_switch_time_;

  ros::Timer execution_timer_;

  // ROS subscribers
  ros::Subscriber exploration_start_sub_;
  ros::Subscriber key_pose_sub_;
  ros::Subscriber state_estimation_sub_;
  ros::Subscriber registered_scan_sub_;
  ros::Subscriber terrain_map_sub_;
  ros::Subscriber terrain_map_ext_sub_;
  ros::Subscriber coverage_boundary_sub_;
  ros::Subscriber viewpoint_boundary_sub_;
  ros::Subscriber nogo_boundary_sub_;
  ros::Subscriber mobility_boundary_sub_;
  ros::Subscriber virtual_obstacle_sub_;
  ros::Subscriber stuck_point_sub_;
  ros::Subscriber covered_point_sub_;
  ros::Subscriber unexplored_point_sub_;
  ros::Subscriber aggressive_sub_;
  ros::Subscriber pause_sub_;
  ros::Subscriber nav_goal_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber free_paths_sub_;

  // Coordination
  ros::Publisher exploration_info_pub_;
  ros::Publisher robot_position_pub_;

  std::vector<ros::Subscriber> robot_position_subscribers_;

  // ROS publishers
  ros::Publisher global_path_full_publisher_;
  ros::Publisher global_path_publisher_;
  ros::Publisher local_tsp_path_publisher_;
  ros::Publisher exploration_path_publisher_;
  ros::Publisher waypoint_pub_;
  ros::Publisher exploration_finish_pub_;
  ros::Publisher robot_exploration_finish_pub_;
  ros::Publisher runtime_breakdown_pub_;
  ros::Publisher runtime_pub_;
  ros::Publisher momentum_activation_count_pub_;
  ros::Publisher next_global_subspace_position_pub_;
  ros::Publisher planner_aggressiveness_pub_;
  ros::Publisher global_comms_relay_nav_path_publisher_;
  ros::Publisher local_comms_relay_nav_path_publisher_;
  ros::Publisher relay_comms_publisher_;
  ros::Publisher vrp_longest_route_length_publisher_;
  ros::Publisher map_clearing_publisher_;
  // Debug
  ros::Publisher pointcloud_manager_neighbor_cells_origin_pub_;

  // Callback functions
  void ExplorationStartCallback(const std_msgs::Bool::ConstPtr& start_msg);
  void StateEstimationCallback(const nav_msgs::Odometry::ConstPtr& state_estimation_msg);
  void KeyposeCallback(const nav_msgs::Odometry::ConstPtr& keypose_msg);
  void KeyPosePathCallback(const nav_msgs::Path::ConstPtr& keypose_path_msg);
  void RegisteredScanCallback(const sensor_msgs::PointCloud2ConstPtr& registered_cloud_msg);
  void TerrainMapCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_map_msg);
  void TerrainMapExtCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_cloud_large_msg);
  void CoverageBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg);
  void ViewPointBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg);
  void NogoBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg);
  void MobilityBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg);
  void VirtualObstacleCallback(const sensor_msgs::PointCloud2ConstPtr& virtual_obstacle_cloud_msg);
  void CoveredPointCallback(const geometry_msgs::PointStamped::ConstPtr& covered_point_msg);
  void UnexploredPointCallback(const geometry_msgs::PointStamped::ConstPtr& unexplored_point_msg);
  void PlannerAggressiveCallback(const std_msgs::Bool::ConstPtr& start_msg);
  void PauseCallback(const std_msgs::Bool::ConstPtr& pause_msg);
  void NavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& nav_goal_msg);
  void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial_pose_msg);
  void FreePathsCallback(const sensor_msgs::PointCloud2ConstPtr& free_paths_msg);

  template <class PCLPointType>
  void RotatePointCloud(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    for (auto& point : cloud->points)
    {
      double tmp_z = point.z;
      point.z = point.y;
      point.y = point.x;
      point.x = tmp_z;
    }
  }
  void SendInitialWaypoint();
  void UpdateKeyposeGraph();
  int UpdateViewPoints();
  void UpdateViewPointCoverage();
  void UpdateRobotViewPointCoverage();
  void UpdateCoveredAreas(int& uncovered_point_num, int& uncovered_frontier_point_num);
  void UpdateVisitedPositions();
  void UpdateGlobalRepresentation();
  void GlobalPlanning(std::vector<int>& global_cell_tsp_order, exploration_path_ns::ExplorationPath& global_path);
  bool OthersFinishedExploring(const std::vector<tare::Robot>& robots, std::vector<bool>& finished_exploring);
  void PublishGlobalPlanningVisualization(const exploration_path_ns::ExplorationPath& global_path,
                                          const exploration_path_ns::ExplorationPath& local_path);
  void LocalPlanning(int uncovered_point_num, int uncovered_frontier_point_num,
                     const exploration_path_ns::ExplorationPath& global_path,
                     exploration_path_ns::ExplorationPath& local_path);
  void LocalPlanningToFollowGlobalPath(const exploration_path_ns::ExplorationPath& global_path,
                                       exploration_path_ns::ExplorationPath& local_path);
  bool GetConvoyGlobalPath(exploration_path_ns::ExplorationPath& global_convoy_path);
  void PublishLocalPlanningVisualization(const exploration_path_ns::ExplorationPath& local_path);
  void PublishCommsRelayPath(const exploration_path_ns::ExplorationPath& comms_relay_global_path,
                             const exploration_path_ns::ExplorationPath& comms_relay_local_path);
  exploration_path_ns::ExplorationPath ConcatenateGlobalLocalPath(
      const exploration_path_ns::ExplorationPath& global_path, const exploration_path_ns::ExplorationPath& local_path);

  void PublishRuntime();
  double GetRobotToHomeDistance();
  void PublishExplorationState();
  void PublishWaypoint();
  void GetNextGlobalSubspace(const exploration_path_ns::ExplorationPath& global_path, int next_subspace_index,
                             double distance_from_start);
  bool GetLookAheadPoint(const exploration_path_ns::ExplorationPath& local_path,
                         const exploration_path_ns::ExplorationPath& global_path, Eigen::Vector3d& lookahead_point,
                         bool& follow_local_path_from_start);
  void PrintExplorationStatus(std::string status, bool clear_last_line = true);
  void CountDirectionChange();
  void PublishPlannerAggressiveness();

  // Coordination
  void PublishExplorationInfo();
  // void SetCommsConfig();

  void SaveKeyposeGraphToFile();
};

}  // namespace sensor_coverage_planner_3d_ns
