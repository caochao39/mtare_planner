/**
 * @file grid_world.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid world
 * @version 0.1
 * @date 2019-11-06
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <memory>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <grid/grid.h>
#include <utils/graph/graph.h>
#include <utils/knowledge_base.h>
#include <utils/pointcloud_utils.h>
#include <tsp_solver/tsp_solver.h>
#include <keypose_graph/keypose_graph.h>
#include <exploration_path/exploration_path.h>
#include <tare_msgs/ExplorationInfo.h>
#include <coordination/robot.h>
#include <coordination/rendezvous_manager.h>
#include <coordination/time_budget_manager.h>

namespace viewpoint_manager_ns
{
class ViewPointManager;
}

namespace grid_world_ns
{
typedef std::vector<std::vector<std::vector<int>>> DistanceMatrix;
typedef std::vector<std::vector<int>> RouteInDMIndices;
typedef std::vector<std::vector<int>> RouteInCellIndices;
struct RoadmapNodeType
{
  int cell_index_;
  geometry_msgs::Point position_;
  bool traversable_;
  RoadmapNodeType();
  RoadmapNodeType(int cell_index, geometry_msgs::Point position, bool traversable = true)
    : cell_index_(cell_index), position_(position), traversable_(traversable)
  {
  }
  friend std::ostream& operator<<(std::ostream& out, const RoadmapNodeType& node);
  friend std::istream& operator>>(std::istream& in, RoadmapNodeType& node);
};

enum class CellStatus
{
  UNSEEN = 0,
  EXPLORING = 1,
  COVERED = 2,
  COVERED_BY_OTHERS = 3,
  NOGO = 4,
  EXPLORING_BY_OTHERS = 5
};

class Cell
{
public:
  explicit Cell(double x = 0.0, double y = 0.0, double z = 0.0);
  explicit Cell(const geometry_msgs::Point& center);
  ~Cell() = default;
  bool IsCellConnected(int cell_ind);
  void AddViewPoint(int viewpoint_ind)
  {
    viewpoint_indices_.push_back(viewpoint_ind);
  }
  void AddGraphNode(int node_ind)
  {
    keypose_graph_node_indices_.push_back(node_ind);
  }
  void ClearViewPointIndices()
  {
    viewpoint_indices_.clear();
  }
  void ClearGraphNodeIndices()
  {
    keypose_graph_node_indices_.clear();
  }
  CellStatus GetStatus()
  {
    return status_;
  }
  void SetStatus(CellStatus status)
  {
    status_ = status;
  }
  std::vector<int> GetViewPointIndices()
  {
    return viewpoint_indices_;
  }
  std::vector<int> GetGraphNodeIndices()
  {
    return keypose_graph_node_indices_;
  }
  geometry_msgs::Point GetPosition()
  {
    return center_;
  }
  void SetPosition(const geometry_msgs::Point& position)
  {
    center_ = position;
  }
  void SetRobotPosition(const geometry_msgs::Point& robot_position)
  {
    robot_position_ = robot_position;
    robot_position_set_ = true;
  }
  void SetKeyposeID(int keypose_id)
  {
    keypose_id_ = keypose_id;
    robot_position_set_ = true;
  }
  bool IsRobotPositionSet()
  {
    return robot_position_set_;
  }
  geometry_msgs::Point GetRobotPosition()
  {
    return robot_position_;
  }
  bool IsCoveredToOthers()
  {
    return covered_to_others_;
  }
  void SetCoveredToOthers(bool covered_to_others)
  {
    covered_to_others_ = covered_to_others;
  }
  bool HasExploringPriority()
  {
    return has_exploring_priority_;
  }
  void SetHasExploringPriority(bool has_exploring_priority)
  {
    has_exploring_priority_ = has_exploring_priority;
  }
  void AddVisitCount()
  {
    visit_count_++;
  }
  int GetVisitCount()
  {
    return visit_count_;
  }
  void AddExploringCount()
  {
    exploring_count_++;
  }
  int GetExploringCount()
  {
    return exploring_count_;
  }
  void Reset();
  int GetKeyposeID()
  {
    return keypose_id_;
  }
  void SetViewPointPosition(const Eigen::Vector3d& position)
  {
    viewpoint_position_ = position;
  }
  Eigen::Vector3d GetViewPointPosition()
  {
    return viewpoint_position_;
  }
  void SetRoadmapConnectionPoint(const Eigen::Vector3d& roadmap_connection_point)
  {
    roadmap_connection_point_ = roadmap_connection_point;
  }
  Eigen::Vector3d GetRoadmapConnectionPoint()
  {
    return roadmap_connection_point_;
  }
  nav_msgs::Path GetPathToKeyposeGraph()
  {
    return path_to_keypose_graph_;
  }
  void SetPathToKeyposeGraph(const nav_msgs::Path& path)
  {
    path_to_keypose_graph_ = path;
  }
  bool IsPathAddedToKeyposeGraph()
  {
    return path_added_to_keypose_graph_;
  }
  void SetPathAddedToKeyposeGraph(bool add_path)
  {
    path_added_to_keypose_graph_ = add_path;
  }
  bool IsRoadmapConnectionPointSet()
  {
    return roadmap_connection_point_set_;
  }
  void SetRoadmapConnectionPointSet(bool set)
  {
    roadmap_connection_point_set_ = set;
  }
  int GetExploringRobotID()
  {
    return exploring_robot_id_;
  }
  void SetExploringRobotID(int robot_id)
  {
    exploring_robot_id_ = robot_id;
  }
  int GetExploredRobotID()
  {
    return explored_robot_id_;
  }
  void SetExploredRobotID(int robot_id)
  {
    explored_robot_id_ = robot_id;
  }
  tare::MobilityType GetMobilityType()
  {
    return mobility_type_;
  }
  void SetMobilityType(tare::MobilityType mobility_type)
  {
    mobility_type_ = mobility_type;
  }
  void InitializeKnowledgeSync(int robot_number)
  {
    for (int i = 0; i < robot_number; i++)
    {
      exploring_synced_count_[i] = 0;
      explored_synced_count_[i] = 0;
    }
  }
  bool IsSynced()
  {
    // return knowledge_base_.IsSynced() && sync_count_ >= kSyncDelay;
    return knowledge_base_.IsSynced();
  }
  int GetSyncedRobotIDs()
  {
    return knowledge_base_.GetSynced();
  }
  void SyncID(int robot_id)
  {
    knowledge_base_.SyncID(robot_id);
  }
  void SyncIDs(int ids)
  {
    knowledge_base_.SyncIDs(ids);
  }
  bool IsSyncedByRobot(int robot_id)
  {
    return knowledge_base_.IsIDSynced(robot_id);
  }
  void AddSyncCount()
  {
    sync_count_++;
    sync_count_ = std::min(1000, sync_count_);
  }
  void ReduceSyncCount()
  {
    sync_count_--;
    sync_count_ = std::max(0, sync_count_);
  }
  void AddSyncedCount(CellStatus cell_status, int robot_id)
  {
    if (cell_status == CellStatus::EXPLORING)
    {
      exploring_synced_count_[robot_id]++;
    }
    else if (cell_status == CellStatus::COVERED)
    {
      explored_synced_count_[robot_id]++;
    }
    else
    {
      ROS_ERROR_STREAM("Unknown cell status " << static_cast<int>(cell_status));
    }
  }

  int GetSyncedCount(int robot_id)
  {
    MY_ASSERT(robot_id >= 0 && robot_id < exploring_synced_count_.size());
    MY_ASSERT(robot_id < explored_synced_count_.size());

    if (status_ == CellStatus::EXPLORING || status_ == CellStatus::EXPLORING_BY_OTHERS)
    {
      return exploring_synced_count_[robot_id];
    }
    else if (status_ == CellStatus::COVERED || status_ == CellStatus::COVERED_BY_OTHERS || covered_to_others_)
    {
      return explored_synced_count_[robot_id];
    }
    else
    {
      return std::max(exploring_synced_count_[robot_id], explored_synced_count_[robot_id]);
    }
  }
  void ResetSyncedCount(int robot_id)
  {
    MY_ASSERT(robot_id >= 0 && robot_id < exploring_synced_count_.size());
    MY_ASSERT(robot_id < explored_synced_count_.size());
    exploring_synced_count_[robot_id] = 0;
    explored_synced_count_[robot_id] = 0;
  }

  void ResetSyncCount()
  {
    sync_count_ = 0;
  }
  int GetSyncCount()
  {
    return sync_count_;
  }
  void ResetSyncedRobots()
  {
    knowledge_base_.Reset();
  }
  void SetVisitedForRelayComms(int robot_id)
  {
    visited_for_relay_comms_.SyncID(robot_id);
  }
  bool IsVisitedForRelayComms(int robot_id)
  {
    return visited_for_relay_comms_.IsIDSynced(robot_id);
  }
  std::map<int, int> GetExploringSyncedCount()
  {
    return exploring_synced_count_;
  }
  std::map<int, int> GetExploredSyncedCount()
  {
    return explored_synced_count_;
  }
  void IncreaseUpdateID()
  {
    update_id_++;
  }
  int GetUpdateID()
  {
    return update_id_;
  }
  void SetUpdateID(int update_id)
  {
    update_id_ = update_id;
  }
  // Delay for sync
  const static int kSyncDelay = 5;

private:
  // Cell status
  CellStatus status_;
  // The center location of this cell.
  geometry_msgs::Point center_;
  // Position of the robot where this cell is first observed and turned EXPLORING
  geometry_msgs::Point robot_position_;
  // Whether the robot position has been set for this cell
  bool robot_position_set_;
  // Sent to others as covered cells
  bool covered_to_others_;
  // The current robot has the priority to explore this cell first
  bool has_exploring_priority_;
  // Number of times the cell is visited by the robot
  int visit_count_;
  // Number of times the cell is turned to exploring state
  int exploring_count_;
  // Indices of the viewpoints within this cell.
  std::vector<int> viewpoint_indices_;
  // Indices of connected keypose graph nodes
  std::vector<int> keypose_graph_node_indices_;
  // Whether this cell is in the planning horizon, which consists of nine cells around the robot.
  bool in_horizon_;
  // ID of the keypose where viewpoints in this cell can be observed
  int keypose_id_;
  // Position of the highest score viewpoint
  Eigen::Vector3d viewpoint_position_;
  // Position for connecting the cell to the global roadmap
  Eigen::Vector3d roadmap_connection_point_;
  // Path to the nearest keypose on the keypose graph
  nav_msgs::Path path_to_keypose_graph_;
  // If the path has been added to the keypose graph
  bool path_added_to_keypose_graph_;
  // If the roadmap connection point has been added to the cell
  bool roadmap_connection_point_set_;
  // Robot id that is exploring the cell
  int exploring_robot_id_;
  // Robot id that has explored the cell
  int explored_robot_id_;
  // Number of updates
  int update_id_;
  // Mobility type
  tare::MobilityType mobility_type_;
  // Sync count
  int sync_count_;
  // Whether the cell status has been synced/known  by certain robot
  tare::KnowledgeBase knowledge_base_;
  // Whether the cell has been visited for relaying comms to certain robot
  tare::KnowledgeBase visited_for_relay_comms_;
  std::map<int, int> exploring_synced_count_;
  std::map<int, int> explored_synced_count_;
};

class GridWorld
{
public:
  explicit GridWorld(ros::NodeHandle& nh);
  // explicit GridWorld(int row_num = 1, int col_num = 1, int level_num = 1, double cell_size = 6.0,
  //                    double cell_height = 6.0, int nearby_grid_num = 5);
  ~GridWorld() = default;
  void ReadParameters(ros::NodeHandle& nh);
  void UpdateNeighborCells(const geometry_msgs::Point& robot_position);
  bool IsNeighbor(int cell_ind);
  void UpdateRobotPosition(const geometry_msgs::Point& robot_position);
  void UpdateCellKeyposeGraphNodes(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  int GetMinAddPointNum()
  {
    return kMinAddPointNumSmall;
  }
  int GetMinAddFrontierPointNum()
  {
    return kMinAddFrontierPointNum;
  }
  geometry_msgs::Point GetOrigin()
  {
    // return origin_;
    Eigen::Vector3d origin = subspaces_->GetOrigin();
    geometry_msgs::Point geo_origin;
    geo_origin.x = origin.x();
    geo_origin.y = origin.y();
    geo_origin.z = origin.z();
    return geo_origin;
  }
  int GetCellNumber()
  {
    return subspaces_->GetCellNumber();
  }
  int sub2ind(const Eigen::Vector3i& sub)
  {
    return subspaces_->Sub2Ind(sub);
  }
  int sub2ind(int row_idx, int col_idx, int level_idx)
  {
    return subspaces_->Sub2Ind(row_idx, col_idx, level_idx);
  }
  Eigen::Vector3i ind2sub(int ind)
  {
    return subspaces_->Ind2Sub(ind);
  }
  void ind2sub(int ind, int& row_idx, int& col_idx, int& level_idx)
  {
    Eigen::Vector3i sub = subspaces_->Ind2Sub(ind);
    row_idx = sub.x();
    col_idx = sub.y();
    level_idx = sub.z();
  }
  bool SubInBound(const Eigen::Vector3i& sub)
  {
    return subspaces_->InRange(sub);
  }
  bool SubInBound(int row_idx, int col_idx, int level_idx)
  {
    return subspaces_->InRange(Eigen::Vector3i(row_idx, col_idx, level_idx));
  }
  bool IndInBound(int ind)
  {
    return subspaces_->InRange(ind);
  }
  // Get the cell index where the robot is currently in.
  bool AreNeighbors(int cell_ind1, int cell_ind2);
  int GetCellInd(double qx, double qy, double qz);
  int GetCellInd(const geometry_msgs::Point& position);
  void GetCellSub(int& row_idx, int& col_idx, int& level_idx, double qx, double qy, double qz);
  Eigen::Vector3i GetCellSub(const Eigen::Vector3d& point);
  // Get the visualization markers for Rviz display.
  void GetMarker(visualization_msgs::Marker& marker);
  // Getthe visualization markers for roadmap
  void GetRoadmapMarker(visualization_msgs::Marker& node_marker, visualization_msgs::Marker& edge_marker);
  // Get the visualization pointcloud for debugging purpose
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud);
  void GetLowExploringPriorityCellsVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud);

  bool Initialized()
  {
    return initialized_;
  }
  void SetUseKeyposeGraph(bool use_keypose_graph)
  {
    use_keypose_graph_ = use_keypose_graph;
  }
  bool UseKeyposeGraph()
  {
    return use_keypose_graph_;
  }

  void AddViewPointToCell(int cell_ind, int viewpoint_ind);
  void AddGraphNodeToCell(int cell_ind, int node_ind);
  void ClearCellViewPointIndices(int cell_ind);
  std::vector<int> GetCellViewPointIndices(int cell_ind);
  std::vector<int> GetNeighborCellIndices()
  {
    return neighbor_cell_indices_;
  };
  void GetNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, const Eigen::Vector3i& neighbor_range,
                              std::vector<int>& neighbor_indices);
  void GetNeighborCellIndices(const geometry_msgs::Point& position, const Eigen::Vector3i& neighbor_range,
                              std::vector<int>& neighbor_indices);
  void GetDirectNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, std::vector<int>& neighbor_indices);
  void GetExploringCellIndices(std::vector<int>& exploring_cell_indices);
  CellStatus GetCellStatus(int cell_ind);
  void SetCellStatus(int cell_ind, CellStatus status);
  void SetCellStatus(int cell_ind, CellStatus status, int robot_id);
  geometry_msgs::Point GetCellPosition(int cell_ind);
  void SetCellRobotPosition(int cell_ind, const geometry_msgs::Point& robot_position);
  geometry_msgs::Point GetCellRobotPosition(int cell_ind);

  void SetCellMobilityType(int cell_ind, tare::MobilityType mobility_type);
  tare::MobilityType GetCellMobilityType(int cell_ind);

  void SetCellExploringRobotID(int cell_ind, int robot_id);
  int GetCellExploringRobotID(int cell_ind);

  void SetCellExploredRobotID(int cell_ind, int robot_id);
  int GetCellExploredRobotID(int cell_ind);

  int GetCellSyncedRobotIDs(int cell_ind);
  int GetCellSyncCount(int cell_ind);
  void ResetCellSyncCount(int cell_ind);
  void ResetCellSync(int cell_ind);

  std::map<int, int> GetCellExploringSyncedCount(int cell_ind);
  std::map<int, int> GetCellExploredSyncedCount(int cell_ind);

  void IncreaseCellUpdateID(int cell_ind);
  int GetCellUpdateID(int cell_ind);
  void SetCellUpdateID(int cell_ind, int update_id);

  void CellAddVisitCount(int cell_ind);
  int GetCellVisitCount(int cell_ind);
  void CellAddExploringCount(int cell_ind);
  int GetCellExploringCount(int cell_ind);
  int GetCellMaxExploringCount()
  {
    return kMaxExploringCount;
  }
  bool IsRobotPositionSet(int cell_ind);
  bool IsCellTraversable(int cell_ind);
  bool IsCellSynced(int cell_ind);
  bool IsCellSyncedByRobot(int cell_ind, int robot_id);
  bool IsCellVisitedForRelayComms(int cell_ind, int robot_id);
  void SetCellVisitedForRelayComms(int cell_ind, int robot_id);
  bool IsCellCoveredToOthers(int cell_ind);
  void SetCellCoveredToOthers(int cell_ind, bool covered_to_others);
  bool IsCellHasExploringPriority(int cell_ind);
  void SetCellHasExploringPriority(int cell_ind, bool has_exploring_priority);
  void Reset();
  bool IsEdgeTraversable(int from_cell_id, int to_cell_id);
  void SetEdgeNontraversable(int from_cell_id, int to_cell_id);
  int GetCellStatusCount(grid_world_ns::CellStatus status);
  void UpdateMobilityBoundary(const std::vector<geometry_msgs::Polygon>& mobility_boundary);
  void UpdateLocalCellMobilityType();
  void UpdateGlobalCellStatus(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  void UpdateLocalCellStatus(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);
  void UpdateLocalExploringCellIDs(const exploration_path_ns::ExplorationPath& local_path, bool follow_from_start);
  void UpdateRobotsForConvoy(std::vector<tare::Robot>& robots);
  void UpdateLocalCellVisitedForRelayComms();
  void AddRoadmapNode(int cell_ind, const geometry_msgs::Point& cell_position, tare::MobilityType cell_mobility_type);
  void AddRoadmapTwoWayEdge(int from_cell_ind, int to_cell_ind);
  void GetExploringCellIDsAndPositions(const std::vector<tare::Robot>& robots,
                                       const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  void GetDistanceMatricesWithTraversability(const std::vector<int>& exploring_cell_ids,
                                             const std::vector<tare::Robot>& robots,
                                             const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                             DistanceMatrix& distance_matrices);
  void GetDistanceMatricesNoComms(const std::vector<int>& exploring_cell_ids, const std::vector<tare::Robot>& robots,
                                  const DistanceMatrix& distance_matrices_with_traversability,
                                  DistanceMatrix& distance_matrices_no_comms);
  void GetDistanceMatricesAssumeComms(const std::vector<int>& exploring_cell_ids,
                                      const std::vector<tare::Robot>& robots,
                                      const DistanceMatrix& distance_matrices_with_traversability,
                                      DistanceMatrix& distance_matrices_assume_comms);
  void GetDistanceMatrixForCommsRelay(const DistanceMatrix& distance_matrices_with_traversability,
                                      const std::vector<tare::Robot>& in_comms_robots,
                                      const std::vector<int>& sampled_node_indices,
                                      DistanceMatrix& distance_matrices_comms_relay);
  // void GetInitialRoutes(int robot_num, const std::vector<int>& exploring_cell_ids,
  //                       std::vector<std::vector<int>>& initial_routes);
  void GetInitialRoutes(const std::vector<tare::Robot>& robots, const std::vector<int>& exploring_cell_ids,
                        const DistanceMatrix& distance_matrices, RouteInDMIndices& initial_routes);

  void GetInitialRoutesFromVRPSolution(const std::vector<int>& exploring_cell_ids,
                                       const DistanceMatrix& distance_matrices, const RouteInDMIndices& vrp_solution,
                                       RouteInDMIndices& initial_routes);
  void ConvertInitialRouteToVRPPlan(const RouteInDMIndices& initial_route, RouteInDMIndices& vrp_plan);
  int ComputeVRPSolutionCost(const RouteInDMIndices& vrp_solution, const DistanceMatrix& distance_matrices,
                             int& dropped_node_num, int& max_route_cost, std::vector<int>& per_route_cost,
                             bool no_cell_to_cell_distance_offset = false);
  tare::VRPCost CombineCommsRelayCost(const RouteInDMIndices& relay_comms_vrp, const RouteInDMIndices& assume_comms_vrp,
                                      const DistanceMatrix& distance_matrices, bool print = false);
  exploration_path_ns::ExplorationPath
  SolveGlobalVRP(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                 std::vector<tare::Robot>& robots, std::vector<int>& ordered_cell_indices,
                 const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph = nullptr);
  exploration_path_ns::ExplorationPath PlanGlobalConvoy(
      std::vector<tare::Robot>& robots, const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  void SampleNodesFromOutOfCommsRobotRoutes(const DistanceMatrix& distance_matrices_with_traversability,
                                            const std::vector<tare::Robot>& robots, const std::vector<bool>& is_sampled,
                                            std::vector<std::pair<int, int>>& time_windows,
                                            std::vector<int>& sampled_node_indices);
  void CombineArrivalProbability(const std::vector<std::vector<int>>& sampled_cell_ids_all_robots,
                                 const std::vector<std::vector<std::vector<double>>>& arrival_probability_all_robots,
                                 std::vector<int>& unique_sampled_cell_ids,
                                 std::vector<std::vector<double>>& combined_arrival_probability);
  void ComputeArrivalProbability(int robot_id, const std::vector<int>& ordered_cell_ids,
                                 const DistanceMatrix& distance_matrix, double time_resolution, int time_steps,
                                 std::vector<int>& sampled_cell_ids,
                                 std::vector<std::vector<double>>& arrival_probability);
  void ComputePursuitDistanceMatrix(const std::vector<tare::Robot>& robots, const std::vector<int>& pursuit_robot_ids,
                                    const std::vector<int>& pursuit_cell_ids,
                                    const DistanceMatrix& distance_matrices_with_traversability, double time_resolution,
                                    std::vector<std::vector<int>>& pursuit_distance_matrix);
  void ComputeArrivalProbabilityOutOfCommsRobotRoutes(const DistanceMatrix& distance_matrices_with_traversability,
                                                      const std::vector<tare::Robot>& robots,
                                                      const std::vector<bool>& is_sampled, double time_resolution,
                                                      int max_time_steps, std::vector<int>& pursuit_cell_ids,
                                                      std::vector<std::vector<double>>& arrival_probability);

  bool SampleOutOfCommsRobotIndices(const std::vector<tare::Robot>& robots, int sample_num,
                                    std::vector<std::vector<bool>>& is_sampled);
  std::pair<int, int> SampleNodeFromRoute(int robot_index, const std::vector<int>& robot_route,
                                          const DistanceMatrix& distance_matrices);

  inline void SetCurKeyposeGraphNodeInd(int node_ind)
  {
    cur_keypose_graph_node_ind_ = node_ind;
  }
  inline void SetCurKeyposeGraphNodePosition(geometry_msgs::Point node_position)
  {
    cur_keypose_graph_node_position_ = node_position;
  }
  inline void SetCurKeyposeID(int keypose_id)
  {
    cur_keypose_id_ = keypose_id;
  }

  inline void SetCurKeypose(const Eigen::Vector3d& cur_keypose)
  {
    cur_keypose_ = cur_keypose;
  }
  int GetCellKeyposeID(int cell_ind);
  void SetHomePosition(const Eigen::Vector3d& home_position)
  {
    home_position_ = home_position;
    set_home_ = true;
  }
  bool HomeSet()
  {
    return set_home_;
  }
  bool IsReturningHome()
  {
    return return_home_;
  }
  bool NoExploringCells()
  {
    return global_exploring_cell_indices_.empty();
  }
  void GetCellViewPointPositions(std::vector<Eigen::Vector3d>& viewpoint_positions);
  bool HasFrontierViewpoints(int cell_ind,
                             const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);
  bool HasKeyposeNodes(int cell_ind, const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  void
  SetRoadmapConnectionPointForCells(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                    const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  void UpdateKeyposeGraph(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                          const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  void UpdateRoadmap(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                     const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph, bool following_global_path);
  void UpdateKeyposeGraphAndRoadmap(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                    const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                    bool following_global_path);
  bool PathValid(const nav_msgs::Path& path, int from_cell_ind, int to_cell_ind);
  bool HasDirectKeyposeGraphConnection(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                       const Eigen::Vector3d& start_position, const Eigen::Vector3d& goal_position);
  void GetExplorationInfo(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                          tare_msgs::ExplorationInfo& exploration_info);
  void GetOrderedExploringCellIndices(RouteInCellIndices& ordered_cell_indices);
  void SetSelfMobilityType(tare::MobilityType mobility_type)
  {
    self_mobility_type_ = mobility_type;
  }
  std::string GetCellIDLabel(int cell_id);
  void SyncCellWithIDs(int cell_id, int ids, CellStatus cell_status);
  void SyncCellWithIDs(int cell_id, int ids);

  int GetRobotNumber()
  {
    return kRobotNum;
  }
  static void DecodeRoadmapEdgeWeight(double weight, tare::MobilityType& from_mobility_type,
                                      tare::MobilityType& to_mobility_type);
  bool RelayComms()
  {
    return relay_comms_ && !relay_comms_global_plan_.empty() && relay_comms_global_plan_[kRobotID].size() > 2;
  }
  bool GoToRendezvous()
  {
    return go_to_rendezvous_;
  }
  void SaveGlobalPlanningStatusToFile(const std::vector<tare::Robot>& robots);
  void UpdateNeighboringCellPriority(const std::vector<tare::Robot>& robots);
  int GetGlobalVRPLongestPathLength()
  {
    return std::min(vrp_cost_.longest_route_length_, 1000000);
  }
  int GetSelfExploringCellsCount();
  void VRPSolutionToInitialGuess(RouteInDMIndices& vrp_solution);
  void InitialGuessToVRPSolution(RouteInDMIndices& initial_guess);
  geometry_msgs::Point GetRendezvousPosition()
  {
    // std::cout << "rendezvous cell id: " << rendezvous_manager_.GetCurrentRendezvousCellID() << std::endl;
    return GetCellPosition(rendezvous_manager_.GetCurrentRendezvousCellID());
  }
  bool WaitForOtherRobots()
  {
    return wait_;
  }
  void ResetTimeBudget()
  {
    int prev_time_budget = time_budget_manager_.GetTimeBudget();
    ROS_INFO_STREAM(misc_utils_ns::ColoredText("Reset time budget to " + std::to_string(prev_time_budget * 2),
                                               misc_utils_ns::TextColor::CYAN));
    time_budget_manager_.SetTimeBudget(prev_time_budget * 2);
  }
  bool TimeToGoHome()
  {
    return time_budget_manager_.TimesUp();
  }
  int GetRobotCellID()
  {
    return GetCellInd(robot_position_);
  }
  Eigen::Vector3i GetRobotCellSub()
  {
    return subspaces_->Ind2Sub(GetCellInd(robot_position_));
  }
  void LoadRoadmapFromFile();

private:
  bool CheckDistanceMatrixSize(const std::vector<std::vector<int>>& distance_matrix, int dim);
  bool IsConnectedOnKeyposeGraph(int cell_ind, const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                 geometry_msgs::Point& connected_point);
  bool IsConnectedOnRoadmap(int cell_ind);
  int CellToCellDistance(int from_cell_ind, int to_cell_ind,
                         const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph = nullptr);
  tare::VRPCost SolveVRP(const DistanceMatrix& distance_matrices, const RouteInDMIndices& initial_routes,
                         RouteInDMIndices& vrp_solution);

  std::pair<int, int> SolveVRPWithTimeWindowConstraint(int robot_num, const DistanceMatrix& distance_matrices,
                                                       const std::vector<std::pair<int, int>>& time_windows,
                                                       const std::vector<bool>& is_sampled,
                                                       const std::vector<int>& sampled_node_indices,
                                                       const RouteInDMIndices& initial_routes,
                                                       RouteInCellIndices& ordered_exploring_cell_ids);
  exploration_path_ns::ExplorationPath
  GetReturnHomePath(const geometry_msgs::Point& global_path_robot_position,
                    const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  exploration_path_ns::ExplorationPath
  GetGlobalExplorationPath(const std::vector<int>& ordered_exploring_cell_ids,
                           const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  geometry_msgs::Point
  GetGlobalPathRobotPosition(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                             const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  void PrintDistanceMatrix(int robot_num, const std::vector<int>& cell_ids, const DistanceMatrix& distance_matrices);
  void AddPathSegment(int from_cell_id, int to_cell_id,
                      const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                      exploration_path_ns::ExplorationPath& global_path);
  void UpdateRoadmapNodeTraversability(tare::MobilityType robot_mobility_type);
  double EncodeRoadmapEdgeWeight(tare::MobilityType from_mobility_type, tare::MobilityType to_mobility_type);

  void AddToAlmostCovered(int cell_ind);
  void RemoveFromAlmostCovered(int cell_ind);
  void GetInCommsRobotIDs(const std::vector<tare::Robot>& robots, std::vector<int>& in_comms_robot_ids);
  void SolveTOPwTVP(const std::vector<int>& pursuit_robot_ids, const std::vector<int>& pursuit_cell_ids,
                    const std::vector<std::vector<double>>& arrival_probability,
                    const std::vector<std::vector<int>>& pursuit_distance_matrix, int time_budget,
                    std::vector<std::vector<int>>& pursuit_paths_in_dm_indices);
  void ComputePursuitRoute(const DistanceMatrix& distance_matrices_with_traversability,
                           const std::vector<tare::Robot>& robots, const std::vector<bool>& is_sampled,
                           RouteInDMIndices& relay_comms_TOPwTVP_solution);
  bool SolveVRPAssumeComms(const std::vector<int>& exploring_cell_ids, const std::vector<tare::Robot>& robots,
                           const tare::VRPCost& no_comms_cost, const RouteInDMIndices& no_comms_vrp_solution,
                           const DistanceMatrix& distance_matrices_with_traversability,
                           const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                           RouteInDMIndices& assume_comms_vrp_solution, RouteInDMIndices& relay_comms_vrp_solution);
  void VRPSolutionToOrderedCellIDs(const std::vector<tare::Robot>& robots, const std::vector<int>& exploring_cell_ids,
                                   const RouteInDMIndices& vrp_solution, RouteInCellIndices& ordered_cell_ids);
  tare_msgs::Cell ToCellMsg(int cell_ind, CellStatus cell_status);
  int GetLongestRouteDistance(const DistanceMatrix& distance_matrices, const RouteInDMIndices& vrp_solution,
                              const std::vector<bool>& is_sampled, const std::vector<int>& sampled_node_indices);
  void PrintVRPSolution(const RouteInDMIndices& vrp_solution);
  void CheckLostRobot(std::vector<tare::Robot>& robots);
  bool HasKnowledgeToShare(const std::vector<tare::Robot>& robots);
  void GetRobotsInComms(const std::vector<tare::Robot>& robots, std::vector<tare::Robot>& robots_in_comms);
  tare::VRPCost GetInitialRoute(const tare::VRPPlan& vrp_plan, const std::vector<int>& exploring_cell_ids,
                                const DistanceMatrix& distance_matrices, RouteInDMIndices& min_cost_routes);
  void GetPeerRobotExploringCellIDs(const std::vector<tare::Robot>& robots,
                                    std::vector<int>& peer_robot_local_exploring_cell_ids);

  tare::VRPCost AugmentVRPPlanToIncludeAllExploringCells(const RouteInCellIndices& ordered_cell_ids,
                                                         const std::vector<int>& exploring_cell_ids,
                                                         const std::vector<bool>& visited,
                                                         const DistanceMatrix& distance_matrices,
                                                         RouteInCellIndices& augmented_ordered_cell_ids);
  void AddNodeToVRPPlan(int to_add_node_index, const DistanceMatrix& distance_matrices,
                        RouteInDMIndices& min_cost_routes);

  void RouteInCellIndicesToDistanceMatricesIndices(const RouteInCellIndices& ordered_cell_ids,
                                                   RouteInDMIndices& routes);

  void PrintVRPSolutionStats(const std::string& route_name, const RouteInDMIndices& vrp_solution,
                             const DistanceMatrix& distance_matrices, bool no_cell_to_cell_distance_offset = false);
  void CheckNotVisitedCells(const RouteInDMIndices& global_plan);
  tare::VRPCost GetPeerRobotsAssumeCommsSolution(const std::vector<tare::Robot>& robots,
                                                 const DistanceMatrix& distance_matrices,
                                                 std::vector<int32_t>& sampled_numbers, int32_t& sampled_number,
                                                 RouteInDMIndices& other_robot_relay_comms_vrp_solution,
                                                 RouteInDMIndices& other_robot_assume_comms_vrp_solution);
  tare::VRPCost GetRobotAssumeCommsSolution(const std::vector<tare::Robot>& robots, const tare::VRPPlan vrp_plan,
                                            const DistanceMatrix& distance_matrices, int32_t& sampled_number,
                                            RouteInDMIndices& robot_relay_comms_vrp_solution,
                                            RouteInDMIndices& robot_assume_comms_vrp_solution);
  int32_t SampleRobotVectorToInt(const std::vector<bool>& sample_robot_vector);
  void SampleRobotIntToVector(int32_t sample_robot_int, std::vector<bool>& sample_robot_vector);
  void SampleRobotIntToRobotIDs(int32_t sample_robot_int, std::vector<int>& robot_ids);
  bool OrderedCellIDsToRelayCommsVRPSolution(const RouteInCellIndices& ordered_cell_ids, int32_t sampled_number,
                                             const std::vector<tare::Robot>& robots,
                                             RouteInDMIndices& relay_comms_vrp_solution);
  void OrderedCellIDsToAssumeCommsVRPSolution(const RouteInCellIndices& ordered_cell_ids,
                                              RouteInDMIndices& assume_comms_vrp_solution);
  bool PlanForRendezvous(const std::vector<tare::Robot>& robots, const DistanceMatrix& distance_matrices,
                         std::vector<int>& rendezvous_global_plan);
  bool AllRobotsInComms(const std::vector<tare::Robot>& robots);
  bool AllRobotsSyncedWithRendezvous(const std::vector<tare::Robot>& robots);
  bool ConnectedToRendezvous(const std::vector<tare::Robot>& robots, int rendezvous_cell_id);
  void SetCommsConfig();

  void VisualizeArrivalProbability(
      const std::vector<int>& pursuit_cell_ids, const std::vector<std::vector<double>>& arrival_probability,
      std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>& arrival_probability_cloud);
  void VisualizePursuitPaths(const std::vector<int>& pursuit_robot_ids, const std::vector<int>& pursuit_cell_ids,
                             const std::vector<tare::Robot>& robots,
                             const std::vector<std::vector<int>>& pursuit_distance_matrix,
                             std::vector<std::vector<int>>& pursuit_paths);
  void ConvertToRouteInDMIndices(const std::vector<int>& pursuit_robot_ids, const std::vector<int>& pursuit_cell_ids,
                                 const std::vector<std::vector<int>>& TOPwTVR_paths,
                                 RouteInDMIndices& relay_comms_TOPwTVP_solution);

  ros::NodeHandle& nh_;

  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> arrival_probability_cloud_;
  std::vector<ros::Publisher> TOPwTVR_path_publishers_;

  int kRobotID;
  int kRowNum;
  int kColNum;
  int kLevelNum;
  double kCellSize;
  double kCellHeight;
  int kCellToCellDistanceOffset;
  int KNearbyGridNum;
  int kMinAddPointNumSmall;
  int kMinAddPointNumBig;
  int kMinAddFrontierPointNum;
  int kCellExploringToCoveredThr;
  int kCellCoveredToExploringThr;
  int kCellExploringToAlmostCoveredThr;
  int kCellAlmostCoveredToExploringThr;
  int kCellUnknownToExploringThr;
  double kCommsRange;
  bool kRelayComms;
  bool kRendezvous;
  bool kRendezvousTree;
  int kRendezvousType;
  int kRendezvousTimeInterval;
  bool kUseTimeBudget;
  int kInitTimeBudget;
  double kRobotSpeed;
  int kRobotStuckInCurrentCellCountThr;

  const int kMaxExploringCount;

  std::unique_ptr<grid_ns::Grid<Cell>> subspaces_;
  tare::Graph<RoadmapNodeType> roadmap_;
  bool initialized_;
  bool use_keypose_graph_;
  int cur_keypose_id_;
  geometry_msgs::Point robot_position_;
  geometry_msgs::Point origin_;
  std::vector<int> neighbor_cell_indices_;
  std::vector<int> almost_covered_cell_indices_;
  std::vector<int> global_exploring_cell_indices_;
  std::vector<int> local_exploring_cell_indices_;
  RouteInDMIndices no_comms_global_plan_;
  RouteInDMIndices assume_comms_global_plan_;
  RouteInDMIndices relay_comms_global_plan_;
  std::vector<int> rendezvous_global_plan_;
  tare::VRPCost vrp_cost_;
  std::vector<nav_msgs::Path> to_connect_cell_paths_;
  Eigen::Vector3d home_position_;
  Eigen::Vector3d cur_keypose_;
  bool set_home_;
  bool return_home_;
  geometry_msgs::Point cur_keypose_graph_node_position_;
  int cur_keypose_graph_node_ind_;
  int cur_robot_cell_ind_;
  int prev_robot_cell_ind_;
  tare::MobilityType self_mobility_type_;
  int kRobotNum;
  std::string kTestID;
  std::vector<bool> is_sampled_last_iteration_;
  bool relay_comms_;
  bool go_to_rendezvous_;
  bool wait_;
  std::vector<int> relay_comms_robot_ids_;
  int convoy_robot_id_;
  std::unordered_map<int, int> cell_id_to_dm_index_;
  tare::RendezvousManager rendezvous_manager_;
  tare::TimeBudgetManager time_budget_manager_;
  int robot_in_current_cell_count_;
  bool robot_stuck_in_current_cell_;
  std::vector<bool> visited_for_convoy_;
  bool wait_at_rendezvous_;
  int wait_at_rendezvous_count_;
};
}  // namespace grid_world_ns
