/**
 * @file grid_world.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid world
 * @version 0.1
 * @date 2019-11-06
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <map>
#include <algorithm>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <utils/misc_utils.h>
#include <viewpoint_manager/viewpoint_manager.h>
#include <grid_world/grid_world.h>
#include <utils/TOPwTVR.h>
#include <utils/pursuit_mdp.h>

namespace grid_world_ns
{
Cell::Cell(double x, double y, double z)
  : in_horizon_(false)
  , robot_position_set_(false)
  , covered_to_others_(false)
  , has_exploring_priority_(true)
  , visit_count_(0)
  , exploring_count_(0)
  , keypose_id_(0)
  , exploring_robot_id_(-1)
  , explored_robot_id_(-1)
  , path_added_to_keypose_graph_(false)
  , roadmap_connection_point_set_(false)
  , viewpoint_position_(Eigen::Vector3d(x, y, z))
  , roadmap_connection_point_(Eigen::Vector3d(x, y, z))
  , mobility_type_(tare::MobilityType::NONE)
  , sync_count_(0)
  , update_id_(0)
{
  center_.x = x;
  center_.y = y;
  center_.z = z;

  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;
  status_ = CellStatus::UNSEEN;
}

Cell::Cell(const geometry_msgs::Point& center) : Cell(center.x, center.y, center.z)
{
}

void Cell::Reset()
{
  status_ = CellStatus::UNSEEN;
  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;
  visit_count_ = 0;
  viewpoint_indices_.clear();
  keypose_graph_node_indices_.clear();
}

RoadmapNodeType::RoadmapNodeType()
{
  cell_index_ = 0;
  position_.x = 0;
  position_.y = 0;
  position_.z = 0;
  traversable_ = true;
}

std::ostream& operator<<(std::ostream& out, const RoadmapNodeType& node)
{
  out << node.cell_index_ << "\t" << node.position_.x << "\t" << node.position_.y << "\t" << node.position_.z;
  return out;
}

std::istream& operator>>(std::istream& in, RoadmapNodeType& node)
{
  in >> node.cell_index_ >> node.position_.x >> node.position_.y >> node.position_.z;
  return in;
}

GridWorld::GridWorld(ros::NodeHandle& nh)
  : initialized_(false)
  , use_keypose_graph_(false)
  , self_mobility_type_(tare::MobilityType::NONE)
  , kMaxExploringCount(2)
  , kRobotStuckInCurrentCellCountThr(5)
  , robot_in_current_cell_count_(0)
  , wait_at_rendezvous_(false)
  , wait_at_rendezvous_count_(0)
  , nh_(nh)
{
  ReadParameters(nh);
  robot_position_.x = 0.0;
  robot_position_.y = 0.0;
  robot_position_.z = 0.0;

  origin_.x = 0.0;
  origin_.y = 0.0;
  origin_.z = 0.0;

  Eigen::Vector3i grid_size(kRowNum, kColNum, kLevelNum);
  Eigen::Vector3d grid_origin(0.0, 0.0, 0.0);
  Eigen::Vector3d grid_resolution(kCellSize, kCellSize, kCellHeight);
  Cell cell_tmp;
  subspaces_ = std::make_unique<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);
  tare::KnowledgeBase::SetTotalNumber(kRobotNum);
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    subspaces_->GetCell(i) = grid_world_ns::Cell();
    subspaces_->GetCell(i).InitializeKnowledgeSync(kRobotNum);
    subspaces_->GetCell(i).SyncID(kRobotID);
  }

  // Initialize roadmap
  roadmap_.SetDistanceFunction([](const RoadmapNodeType& node1, const RoadmapNodeType& node2) {
    return misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(node1.position_, node2.position_);
  });
  roadmap_.SetIDFunction([](const RoadmapNodeType& node) { return node.cell_index_; });
  roadmap_.SetPositionFunction([](const RoadmapNodeType& node) { return node.position_; });
  roadmap_.SetNodeConstraintFunction([](const RoadmapNodeType& node) { return node.traversable_; });

  home_position_.x() = 0.0;
  home_position_.y() = 0.0;
  home_position_.z() = 0.0;

  cur_keypose_graph_node_position_.x = 0.0;
  cur_keypose_graph_node_position_.y = 0.0;
  cur_keypose_graph_node_position_.z = 0.0;

  set_home_ = false;
  return_home_ = false;

  cur_robot_cell_ind_ = -1;
  prev_robot_cell_ind_ = -1;

  is_sampled_last_iteration_.resize(kRobotNum, false);
  visited_for_convoy_.resize(kRobotNum, false);

  relay_comms_ = false;
  go_to_rendezvous_ = false;
  wait_ = false;

  convoy_robot_id_ = kRobotID;

  time_budget_manager_.SetTimeBudget(kInitTimeBudget);

  arrival_probability_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh_, "/arrival_probability_cloud", "map");

  for (int i = 0; i < kRobotNum; i++)
  {
    std::string topic_name = "/TOPwTVR_path" + std::to_string(i);
    ros::Publisher pub = nh.advertise<nav_msgs::Path>(topic_name, 1);
    TOPwTVR_path_publishers_.push_back(pub);
  }
}

void GridWorld::ReadParameters(ros::NodeHandle& nh)
{
  kRobotID = misc_utils_ns::getParam<int>(nh, "robot_id", 0);
  kRowNum = misc_utils_ns::getParam<int>(nh, "kGridWorldXNum", 121);
  kColNum = misc_utils_ns::getParam<int>(nh, "kGridWorldYNum", 121);
  kLevelNum = misc_utils_ns::getParam<int>(nh, "kGridWorldZNum", 121);
  int viewpoint_number = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_x", 40);
  double viewpoint_resolution = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 1.0);
  kCellSize = viewpoint_number * viewpoint_resolution / 5;
  kCellHeight = misc_utils_ns::getParam<double>(nh, "kGridWorldCellHeight", 8.0);
  kCellToCellDistanceOffset = misc_utils_ns::getParam<int>(nh, "kCellToCellDistanceOffset", 500);
  KNearbyGridNum = misc_utils_ns::getParam<int>(nh, "kGridWorldNearbyGridNum", 5);
  kMinAddPointNumSmall = misc_utils_ns::getParam<int>(nh, "kMinAddPointNumSmall", 60);
  kMinAddPointNumBig = misc_utils_ns::getParam<int>(nh, "kMinAddPointNumBig", 100);
  kMinAddFrontierPointNum = misc_utils_ns::getParam<int>(nh, "kMinAddFrontierPointNum", 30);
  kCellExploringToCoveredThr = misc_utils_ns::getParam<int>(nh, "kCellExploringToCoveredThr", 1);
  kCellCoveredToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellCoveredToExploringThr", 10);
  kCellExploringToAlmostCoveredThr = misc_utils_ns::getParam<int>(nh, "kCellExploringToAlmostCoveredThr", 10);
  kCellAlmostCoveredToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellAlmostCoveredToExploringThr", 20);
  kCellUnknownToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellUnknownToExploringThr", 1);
  kCommsRange = misc_utils_ns::getParam<double>(nh, "kCommsRange", DBL_MAX);
  kRelayComms = misc_utils_ns::getParam<bool>(nh, "kRelayComms", false);
  kRendezvous = misc_utils_ns::getParam<bool>(nh, "kRendezvous", false);
  kRendezvousTree = misc_utils_ns::getParam<bool>(nh, "kRendezvousTree", false);
  kRendezvousType = misc_utils_ns::getParam<int>(nh, "kRendezvousType", 0);
  kRendezvousTimeInterval = misc_utils_ns::getParam<int>(nh, "kRendezvousTimeInterval", 10);
  kUseTimeBudget = misc_utils_ns::getParam<bool>(nh, "kUseTimeBudget", false);
  kInitTimeBudget = misc_utils_ns::getParam<int>(nh, "kInitTimeBudget", 120);
  kRobotSpeed = misc_utils_ns::getParam<double>(nh, "kRobotSpeed", 2.0);

  kRobotNum = misc_utils_ns::getParam<int>(nh, "kRobotNum", 1);
  kTestID = misc_utils_ns::getParam<std::string>(nh, "kTestID", "0001");

  MY_ASSERT(!kTestID.empty());

  ROS_INFO_STREAM("grid world kTestID: " << kTestID);
  SetCommsConfig();

  std::string robot_num_string = kTestID.substr(2, 2);
  int robot_num_from_test_id = stoi(robot_num_string);
  MY_ASSERT(kRobotNum == robot_num_from_test_id);
}

void GridWorld::UpdateNeighborCells(const geometry_msgs::Point& robot_position)
{
  if (!initialized_)
  {
    initialized_ = true;
    origin_.x = robot_position.x - (kCellSize * kRowNum) / 2;
    origin_.y = robot_position.y - (kCellSize * kColNum) / 2;
    origin_.z = robot_position.z - (kCellHeight * kLevelNum) / 2;
    subspaces_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z));
    // Update cell centers
    for (int i = 0; i < kRowNum; i++)
    {
      for (int j = 0; j < kColNum; j++)
      {
        for (int k = 0; k < kLevelNum; k++)
        {
          Eigen::Vector3d subspace_center_position = subspaces_->Sub2Pos(i, j, k);
          geometry_msgs::Point subspace_center_geo_position;
          subspace_center_geo_position.x = subspace_center_position.x();
          subspace_center_geo_position.y = subspace_center_position.y();
          subspace_center_geo_position.z = subspace_center_position.z();
          subspaces_->GetCell(i, j, k).SetPosition(subspace_center_geo_position);
          subspaces_->GetCell(i, j, k).SetRoadmapConnectionPoint(subspace_center_position);
        }
      }
    }
  }

  // Get neighbor cells
  std::vector<int> prev_neighbor_cell_indices = neighbor_cell_indices_;
  neighbor_cell_indices_.clear();
  int N = KNearbyGridNum / 2;
  int M = 1;
  GetNeighborCellIndices(robot_position, Eigen::Vector3i(N, N, M), neighbor_cell_indices_);

  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    if (std::find(prev_neighbor_cell_indices.begin(), prev_neighbor_cell_indices.end(), cell_ind) ==
        prev_neighbor_cell_indices.end())
    {
      // subspaces_->GetCell(cell_ind).AddVisitCount();
      subspaces_->GetCell(cell_ind).AddVisitCount();
    }
  }
}

bool GridWorld::IsNeighbor(int cell_ind)
{
  return std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), cell_ind) !=
         neighbor_cell_indices_.end();
}

void GridWorld::UpdateRobotPosition(const geometry_msgs::Point& robot_position)
{
  robot_position_ = robot_position;
  int robot_cell_ind = GetCellInd(robot_position_);
  if (cur_robot_cell_ind_ != robot_cell_ind)
  {
    prev_robot_cell_ind_ = cur_robot_cell_ind_;
    cur_robot_cell_ind_ = robot_cell_ind;
    robot_stuck_in_current_cell_ = false;
    robot_in_current_cell_count_ = 0;
  }
  else
  {
    robot_in_current_cell_count_++;
    if (robot_in_current_cell_count_ > kRobotStuckInCurrentCellCountThr)
    {
      robot_stuck_in_current_cell_ = true;
    }
    else
    {
      robot_stuck_in_current_cell_ = false;
    }
  }
  if (!rendezvous_manager_.InitialRendezvousSet())
  {
    rendezvous_manager_.SetCurrentRendezvousCellID(cur_robot_cell_ind_);
    rendezvous_manager_.SetNextrRendezvousCellID(cur_robot_cell_ind_);
  }
}

void GridWorld::UpdateCellKeyposeGraphNodes(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  std::vector<int> keypose_graph_connected_node_indices = keypose_graph->GetConnectedGraphNodeIndices();

  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    {
      subspaces_->GetCell(i).ClearGraphNodeIndices();
    }
  }
  for (const auto& node_ind : keypose_graph_connected_node_indices)
  {
    geometry_msgs::Point node_position = keypose_graph->GetNodePosition(node_ind);
    int cell_ind = GetCellInd(node_position);
    if (subspaces_->InRange(cell_ind))
    {
      {
        subspaces_->GetCell(cell_ind).AddGraphNode(node_ind);
      }
    }
  }
}

bool GridWorld::AreNeighbors(int cell_ind1, int cell_ind2)
{
  Eigen::Vector3i cell_sub1 = subspaces_->Ind2Sub(cell_ind1);
  Eigen::Vector3i cell_sub2 = subspaces_->Ind2Sub(cell_ind2);
  Eigen::Vector3i diff = cell_sub1 - cell_sub2;
  if (std::abs(diff.x()) + std::abs(diff.y()) + std::abs(diff.z()) == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int GridWorld::GetCellInd(double qx, double qy, double qz)
{
  Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
  if (subspaces_->InRange(sub))
  {
    return subspaces_->Sub2Ind(sub);
  }
  else
  {
    return -1;
  }
}

int GridWorld::GetCellInd(const geometry_msgs::Point& position)
{
  return GetCellInd(position.x, position.y, position.z);
}

void GridWorld::GetCellSub(int& row_idx, int& col_idx, int& level_idx, double qx, double qy, double qz)
{
  Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
  row_idx = (sub.x() >= 0 && sub.x() < kRowNum) ? sub.x() : -1;
  col_idx = (sub.y() >= 0 && sub.y() < kColNum) ? sub.y() : -1;
  level_idx = (sub.z() >= 0 && sub.z() < kLevelNum) ? sub.z() : -1;
}

Eigen::Vector3i GridWorld::GetCellSub(const Eigen::Vector3d& point)
{
  return subspaces_->Pos2Sub(point);
}

void GridWorld::GetMarker(visualization_msgs::Marker& marker)
{
  marker.points.clear();
  marker.colors.clear();
  marker.scale.x = kCellSize;
  marker.scale.y = kCellSize;
  marker.scale.z = kCellHeight;

  int exploring_count = 0;
  int covered_count = 0;
  int unseen_count = 0;

  for (int i = 0; i < kRowNum; i++)
  {
    for (int j = 0; j < kColNum; j++)
    {
      for (int k = 0; k < kLevelNum; k++)
      {
        int cell_ind = subspaces_->Sub2Ind(i, j, k);
        geometry_msgs::Point cell_center = subspaces_->GetCell(cell_ind).GetPosition();
        std_msgs::ColorRGBA color;
        bool add_marker = false;
        if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN)
        {
          color.r = 0.0;
          color.g = 0.0;
          color.b = 1.0;
          color.a = 0.1;
          unseen_count++;
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED)
        {
          color.r = 1.0;
          color.g = 1.0;
          color.b = 0.0;
          color.a = 0.1;
          covered_count++;
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
        {
          color.r = 0.0;
          color.g = 1.0;
          color.b = 0.0;
          color.a = 0.1;
          exploring_count++;
          add_marker = true;
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::NOGO)
        {
          color.r = 1.0;
          color.g = 0.0;
          color.b = 0.0;
          color.a = 0.1;
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING_BY_OTHERS)
        {
          color.r = 1.0;
          color.g = 0.0;
          color.b = 1.0;
          color.a = 0.1;
          add_marker = true;
        }
        else
        {
          color.r = 0.8;
          color.g = 0.8;
          color.b = 0.8;
          color.a = 0.1;
        }
        if (add_marker)
        {
          marker.colors.push_back(color);
          marker.points.push_back(cell_center);
        }
      }
    }
  }
}

void GridWorld::GetRoadmapMarker(visualization_msgs::Marker& node_marker, visualization_msgs::Marker& edge_marker)
{
  roadmap_.GetVisualizationMarkers(node_marker, edge_marker);
}

void GridWorld::GetLowExploringPriorityCellsVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud)
{
  vis_cloud->points.clear();
  for (const auto& cell_id : neighbor_cell_indices_)
  {
    if (!IsCellHasExploringPriority(cell_id))
    {
      pcl::PointXYZI point;
      geometry_msgs::Point position = subspaces_->GetCell(cell_id).GetPosition();
      point.x = position.x;
      point.y = position.y;
      point.z = position.z;
      point.intensity = 0;
      vis_cloud->points.push_back(point);
    }
  }
}

void GridWorld::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud)
{
  vis_cloud->points.clear();
  int cell_number = subspaces_->GetCellNumber();
  for (int i = 0; i < cell_number; i++)
  {
    CellStatus cell_status = GetCellStatus(i);
    if (cell_status == CellStatus::COVERED || cell_status == CellStatus::EXPLORING ||
        cell_status == CellStatus::EXPLORING_BY_OTHERS || cell_status == CellStatus::COVERED_BY_OTHERS ||
        (cell_status == CellStatus::UNSEEN && IsCellCoveredToOthers(i)))
    {
      pcl::PointXYZI point;
      geometry_msgs::Point position = subspaces_->GetCell(i).GetPosition();
      point.x = position.x;
      point.y = position.y;
      point.z = position.z;
      point.intensity = static_cast<float>(cell_status);
      if (cell_status == CellStatus::EXPLORING && !IsCellHasExploringPriority(i))
      {
        point.intensity += 0.5;
      }
      vis_cloud->points.push_back(point);
    }
  }
}

void GridWorld::AddViewPointToCell(int cell_ind, int viewpoint_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddViewPoint(viewpoint_ind);
}

void GridWorld::AddGraphNodeToCell(int cell_ind, int node_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddGraphNode(node_ind);
}

void GridWorld::ClearCellViewPointIndices(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).ClearViewPointIndices();
}

std::vector<int> GridWorld::GetCellViewPointIndices(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetViewPointIndices();
}

void GridWorld::GetNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, const Eigen::Vector3i& neighbor_range,
                                       std::vector<int>& neighbor_indices)
{
  neighbor_indices.clear();
  int row_idx = 0;
  int col_idx = 0;
  int level_idx = 0;
  for (int i = -neighbor_range.x(); i <= neighbor_range.x(); i++)
  {
    row_idx = center_cell_sub.x() + i;
    for (int j = -neighbor_range.y(); j <= neighbor_range.y(); j++)
    {
      col_idx = center_cell_sub.y() + j;
      for (int k = -neighbor_range.z(); k <= neighbor_range.z(); k++)
      {
        level_idx = center_cell_sub.z() + k;
        Eigen::Vector3i sub(row_idx, col_idx, level_idx);
        if (subspaces_->InRange(sub))
        {
          int ind = subspaces_->Sub2Ind(sub);
          neighbor_indices.push_back(ind);
        }
      }
    }
  }
}
void GridWorld::GetNeighborCellIndices(const geometry_msgs::Point& position, const Eigen::Vector3i& neighbor_range,
                                       std::vector<int>& neighbor_indices)
{
  Eigen::Vector3i center_cell_sub = GetCellSub(Eigen::Vector3d(position.x, position.y, position.z));

  GetNeighborCellIndices(center_cell_sub, neighbor_range, neighbor_indices);
}

void GridWorld::GetDirectNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, std::vector<int>& neighbor_indices)
{
  neighbor_indices.clear();
  for (int x = -1; x <= 1; x++)
  {
    for (int y = -1; y <= 1; y++)
    {
      for (int z = -1; z <= 1; z++)
      {
        if (std::abs(x) + std::abs(y) + std::abs(z) == 1)
        {
          Eigen::Vector3i neighbor_sub = center_cell_sub + Eigen::Vector3i(x, y, z);
          if (subspaces_->InRange(neighbor_sub))
          {
            int neighbor_ind = subspaces_->Sub2Ind(neighbor_sub);
            neighbor_indices.push_back(neighbor_ind);
          }
        }
      }
    }
  }
}

void GridWorld::GetExploringCellIndices(std::vector<int>& exploring_cell_indices)
{
  exploring_cell_indices.clear();
  for (int i = 0; i < global_exploring_cell_indices_.size(); i++)
  {
    exploring_cell_indices.push_back(global_exploring_cell_indices_[i]);
  }
}

CellStatus GridWorld::GetCellStatus(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetStatus();
}

void GridWorld::SetCellStatus(int cell_ind, grid_world_ns::CellStatus status)
{
  SetCellStatus(cell_ind, status, kRobotID);
}

void GridWorld::SetCellStatus(int cell_ind, CellStatus status, int robot_id)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  CellStatus prev_cell_status = GetCellStatus(cell_ind);
  subspaces_->GetCell(cell_ind).SetStatus(status);
  if (status == CellStatus::EXPLORING || status == CellStatus::EXPLORING_BY_OTHERS)
  {
    subspaces_->GetCell(cell_ind).SetExploringRobotID(robot_id);
    SetCellCoveredToOthers(cell_ind, false);
    if (prev_cell_status != CellStatus::EXPLORING && prev_cell_status != CellStatus::EXPLORING_BY_OTHERS)
    {
      CellAddExploringCount(cell_ind);
    }
  }
  else if (status == CellStatus::COVERED || status == CellStatus::COVERED_BY_OTHERS)
  {
    subspaces_->GetCell(cell_ind).SetExploredRobotID(robot_id);
    SetCellCoveredToOthers(cell_ind, true);
  }
}

geometry_msgs::Point GridWorld::GetCellPosition(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetPosition();
}

void GridWorld::SetCellRobotPosition(int cell_ind, const geometry_msgs::Point& robot_position)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position);
}

geometry_msgs::Point GridWorld::GetCellRobotPosition(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetRobotPosition();
}

void GridWorld::SetCellMobilityType(int cell_ind, tare::MobilityType mobility_type)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetMobilityType(mobility_type);
}
tare::MobilityType GridWorld::GetCellMobilityType(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetMobilityType();
}

void GridWorld::SetCellExploringRobotID(int cell_ind, int robot_id)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetExploringRobotID(robot_id);
}
int GridWorld::GetCellExploringRobotID(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetExploringRobotID();
}

void GridWorld::SetCellExploredRobotID(int cell_ind, int robot_id)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetExploredRobotID(robot_id);
}
int GridWorld::GetCellExploredRobotID(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetExploredRobotID();
}

int GridWorld::GetCellSyncedRobotIDs(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetSyncedRobotIDs();
}

int GridWorld::GetCellSyncCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetSyncCount();
}

void GridWorld::ResetCellSyncCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).ResetSyncCount();
}

void GridWorld::ResetCellSync(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).ResetSyncCount();
  subspaces_->GetCell(cell_ind).ResetSyncedRobots();
  subspaces_->GetCell(cell_ind).SyncID(kRobotID);
}

std::map<int, int> GridWorld::GetCellExploringSyncedCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetExploringSyncedCount();
}

std::map<int, int> GridWorld::GetCellExploredSyncedCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetExploredSyncedCount();
}

void GridWorld::IncreaseCellUpdateID(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).IncreaseUpdateID();
}

int GridWorld::GetCellUpdateID(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetUpdateID();
}

void GridWorld::SetCellUpdateID(int cell_ind, int update_id)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetUpdateID(update_id);
}

void GridWorld::CellAddVisitCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddVisitCount();
}

int GridWorld::GetCellVisitCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetVisitCount();
}

void GridWorld::CellAddExploringCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddExploringCount();
}

int GridWorld::GetCellExploringCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetExploringCount();
}

bool GridWorld::IsRobotPositionSet(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).IsRobotPositionSet();
}

bool GridWorld::IsCellTraversable(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return tare::Traversable(self_mobility_type_, GetCellMobilityType(cell_ind));
}

bool GridWorld::IsCellSynced(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).IsSynced();
}

bool GridWorld::IsCellSyncedByRobot(int cell_ind, int robot_id)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).IsSyncedByRobot(robot_id);
}

bool GridWorld::IsCellVisitedForRelayComms(int cell_ind, int robot_id)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).IsVisitedForRelayComms(robot_id);
}

void GridWorld::SetCellVisitedForRelayComms(int cell_ind, int robot_id)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetVisitedForRelayComms(robot_id);
}

bool GridWorld::IsCellCoveredToOthers(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).IsCoveredToOthers();
}

void GridWorld::SetCellCoveredToOthers(int cell_ind, bool covered_to_others)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetCoveredToOthers(covered_to_others);
}

bool GridWorld::IsCellHasExploringPriority(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).HasExploringPriority();
}

void GridWorld::SetCellHasExploringPriority(int cell_ind, bool has_exploring_priority)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetHasExploringPriority(has_exploring_priority);
}

void GridWorld::Reset()
{
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    subspaces_->GetCell(i).Reset();
  }
}

bool GridWorld::IsEdgeTraversable(int from_cell_id, int to_cell_id)
{
  return roadmap_.GetEdgeDistance(from_cell_id, to_cell_id) < roadmap_.GetEdgeCutOffDistanceThreshold();
}

void GridWorld::SetEdgeNontraversable(int from_cell_id, int to_cell_id)
{
  roadmap_.SetEdgeDistance(from_cell_id, to_cell_id, misc_utils_ns::INF_DISTANCE);
}

int GridWorld::GetCellStatusCount(grid_world_ns::CellStatus status)
{
  int count = 0;
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == status)
    {
      count++;
    }
  }
  return count;
}

void GridWorld::UpdateMobilityBoundary(const std::vector<geometry_msgs::Polygon>& mobility_boundary)
{
  int cell_number = subspaces_->GetCellNumber();

  for (int j = 0; j < mobility_boundary.size(); j++)
  {
    geometry_msgs::Polygon polygon = mobility_boundary[j];
    int in_poly_count = 0;
    for (int i = 0; i < cell_number; i++)
    {
      geometry_msgs::Point cell_center = GetCellPosition(i);
      // TODO: this is a hack
      if (std::abs(cell_center.z) > 3)
      {
        continue;
      }
      if (polygon.points.empty())
      {
        continue;
      }
      tare::MobilityType mobility_type = static_cast<tare::MobilityType>(polygon.points.front().z);
      if (misc_utils_ns::PointInPolygon(cell_center, polygon))
      {
        in_poly_count++;
        SetCellMobilityType(i, mobility_type);
      }
    }
  }
}

void GridWorld::UpdateLocalCellMobilityType()
{
}

void GridWorld::UpdateGlobalCellStatus(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (IsNeighbor(i))
    {
      continue;
    }
    SetCellHasExploringPriority(i, true);
    bool set_to_covered = false;
    CellStatus cell_status = GetCellStatus(i);
    // If an exploring cell is surrounded by covered cells, set it to covered
    if (cell_status == CellStatus::EXPLORING || cell_status == CellStatus::EXPLORING_BY_OTHERS)
    {
      int covered_count = 0;
      Eigen::Vector3i cell_sub = subspaces_->Ind2Sub(i);
      for (int x = -1; x <= 1; x++)
      {
        for (int y = -1; y <= 1; y++)
        {
          if (x == 0 && y == 0)
          {
            continue;
          }
          Eigen::Vector3i neighbor_cell_sub = cell_sub;
          neighbor_cell_sub.x() += x;
          neighbor_cell_sub.y() += y;
          if (subspaces_->InRange(neighbor_cell_sub))
          {
            int neighbor_cell_ind = subspaces_->Sub2Ind(neighbor_cell_sub);
            CellStatus neighbor_cell_status = GetCellStatus(neighbor_cell_ind);
            if (neighbor_cell_status == CellStatus::COVERED || neighbor_cell_status == CellStatus::COVERED_BY_OTHERS ||
                IsCellCoveredToOthers(neighbor_cell_ind))
            {
              covered_count++;
            }
          }
        }
      }
      if (covered_count == 8)
      {
        set_to_covered = true;
      }
    }

    geometry_msgs::Point connected_point;
    if ((cell_status == CellStatus::EXPLORING && !IsConnectedOnKeyposeGraph(i, keypose_graph, connected_point)) ||
        (cell_status == CellStatus::EXPLORING_BY_OTHERS && !IsConnectedOnRoadmap(i)))
    {
      // If an exploring cell is not connected on keypose graph or roadmap, set it to covered
      set_to_covered = true;
    }

    if (set_to_covered)
    {
      SetCellStatus(i, CellStatus::COVERED);
      ResetCellSync(i);
      IncreaseCellUpdateID(i);
      // ROS_INFO_STREAM("Update global cell status: setting " << GetCellIDLabel(i) << " to covered");
    }
  }
}

void GridWorld::UpdateLocalCellStatus(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    subspaces_->GetCell(cell_ind).ClearViewPointIndices();
  }
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    geometry_msgs::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
    Eigen::Vector3i sub =
        subspaces_->Pos2Sub(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));
    if (subspaces_->InRange(sub))
    {
      int cell_ind = subspaces_->Sub2Ind(sub);
      AddViewPointToCell(cell_ind, viewpoint_ind);
      viewpoint_manager->SetViewPointCellInd(viewpoint_ind, cell_ind);
    }
    else
    {
      ROS_ERROR_STREAM("subspace sub out of bound: " << sub.transpose());
    }
  }

  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    CellStatus cell_status = GetCellStatus(cell_ind);
    if (cell_status == CellStatus::COVERED_BY_OTHERS || !IsCellTraversable(cell_ind))
    {
      // TODO: this is only a hack, when a cell is not seen and not traversable, report to others to cover
      if (cell_status == CellStatus::UNSEEN && std::abs(robot_position_.z - GetCellPosition(cell_ind).z) < 2 &&
          roadmap_.HasNode(cell_ind))
      {
        SetCellStatus(cell_ind, CellStatus::EXPLORING_BY_OTHERS);
        SetCellExploringRobotID(cell_ind, kRobotID);
        IncreaseCellUpdateID(cell_ind);
        ResetCellSync(cell_ind);
      }
      continue;
    }

    int candidate_count = 0;
    int selected_viewpoint_count = 0;
    int above_big_threshold_count = 0;
    int above_small_threshold_count = 0;
    int above_frontier_threshold_count = 0;
    int highest_score_viewpoint_ind = -1;
    int highest_score = -1;
    for (const auto& viewpoint_ind : subspaces_->GetCell(cell_ind).GetViewPointIndices())
    {
      MY_ASSERT(viewpoint_manager->IsViewPointCandidate(viewpoint_ind));
      candidate_count++;
      if (viewpoint_manager->ViewPointSelected(viewpoint_ind))
      {
        selected_viewpoint_count++;
      }
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        continue;
      }
      int score = viewpoint_manager->GetViewPointCoveredPointNum(viewpoint_ind);
      int frontier_score = viewpoint_manager->GetViewPointCoveredFrontierPointNum(viewpoint_ind);
      if (score > highest_score)
      {
        highest_score = score;
        highest_score_viewpoint_ind = viewpoint_ind;
      }
      if (score >= kMinAddPointNumSmall)
      {
        above_small_threshold_count++;
      }
      if (score >= kMinAddPointNumBig)
      {
        above_big_threshold_count++;
      }
      if (frontier_score > kMinAddFrontierPointNum)
      {
        above_frontier_threshold_count++;
      }
    }

    bool has_selected_viewpoint = selected_viewpoint_count > 0;
    bool has_candidate = candidate_count > 0;
    bool coverage_below_threshold = above_frontier_threshold_count < kCellExploringToCoveredThr &&
                                    above_small_threshold_count < kCellExploringToCoveredThr;
    bool coverage_above_high_threshold = above_frontier_threshold_count > kCellCoveredToExploringThr ||
                                         above_big_threshold_count > kCellCoveredToExploringThr;
    bool almost_covered = std::find(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(),
                                    cell_ind) != almost_covered_cell_indices_.end();
    bool is_covered_to_others = IsCellCoveredToOthers(cell_ind);

    if (cell_status == CellStatus::UNSEEN)
    {
      if (has_selected_viewpoint)
      {
        // To EXPLORING
        SetCellStatus(cell_ind, CellStatus::EXPLORING);
        RemoveFromAlmostCovered(cell_ind);
        ResetCellSync(cell_ind);
        IncreaseCellUpdateID(cell_ind);
      }
      else if (has_candidate && !is_covered_to_others)
      {
        // To covered to others
        SetCellCoveredToOthers(cell_ind, true);
        subspaces_->GetCell(cell_ind).SetExploredRobotID(kRobotID);
        ResetCellSync(cell_ind);
        IncreaseCellUpdateID(cell_ind);
        // AddToAlmostCovered(cell_ind);
      }
    }
    else if (cell_status == CellStatus::EXPLORING)
    {
      if (has_selected_viewpoint)
      {
        RemoveFromAlmostCovered(cell_ind);
      }
      else if (has_candidate)
      {
        if (coverage_below_threshold)
        {
          // To COVERED
          SetCellStatus(cell_ind, CellStatus::COVERED);
          ResetCellSync(cell_ind);
          IncreaseCellUpdateID(cell_ind);
        }
        else if (!almost_covered)
        {
          // To almost covered
          AddToAlmostCovered(cell_ind);
        }
      }
      else
      {
        if (subspaces_->GetCell(cell_ind).GetVisitCount() == 1 &&
            subspaces_->GetCell(cell_ind).GetGraphNodeIndices().empty())
        {
          SetCellStatus(cell_ind, CellStatus::COVERED);
          ResetCellSync(cell_ind);
          IncreaseCellUpdateID(cell_ind);
        }
        else
        {
          geometry_msgs::Point cell_position = subspaces_->GetCell(cell_ind).GetPosition();
          double xy_dist_to_robot =
              misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(cell_position, robot_position_);
          double z_dist_to_robot = std::abs(cell_position.z - robot_position_.z);
          // TODO: review this criteria
          if (xy_dist_to_robot < kCellSize && z_dist_to_robot < kCellHeight * 0.8)
          {
            SetCellStatus(cell_ind, CellStatus::COVERED);
            ResetCellSync(cell_ind);
            IncreaseCellUpdateID(cell_ind);
          }
        }
      }
    }
    else if (cell_status == CellStatus::EXPLORING_BY_OTHERS)
    {
      if (has_selected_viewpoint && IsCellTraversable(cell_ind))
      {
        SetCellStatus(cell_ind, CellStatus::EXPLORING);
      }
      else if (has_candidate)
      {
        // To almost covered
        AddToAlmostCovered(cell_ind);
      }
      else
      {
        // TODO: if the cell is the next one that the robot is going to pursue and go stuck, set to covered, but
        // probably not here
      }
    }
    else if (cell_status == CellStatus::COVERED)
    {
      // To exploring
      if (coverage_above_high_threshold && GetCellExploringCount(cell_ind) <= kMaxExploringCount)
      {
        SetCellStatus(cell_ind, CellStatus::EXPLORING);
        IncreaseCellUpdateID(cell_ind);
        ResetCellSync(cell_ind);
      }
    }
    else if (cell_status == CellStatus::COVERED_BY_OTHERS)
    {
      // Do nothing
    }
    else if (cell_status == CellStatus::NOGO)
    {
      // Do nothing
    }

    if (cell_status == CellStatus::EXPLORING && has_candidate)
    {
      subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position_);
      subspaces_->GetCell(cell_ind).SetKeyposeID(cur_keypose_id_);
    }
  }
  for (const auto& cell_ind : almost_covered_cell_indices_)
  {
    if (!IsNeighbor(cell_ind))
    {
      if (!IsCellCoveredToOthers(cell_ind))
      {
        ResetCellSync(cell_ind);
        IncreaseCellUpdateID(cell_ind);
      }
      SetCellStatus(cell_ind, CellStatus::COVERED);
      RemoveFromAlmostCovered(cell_ind);
    }
  }
}

void GridWorld::UpdateLocalExploringCellIDs(const exploration_path_ns::ExplorationPath& local_path,
                                            bool follow_from_start)
{
  local_exploring_cell_indices_.clear();
  if (local_path.nodes_.empty())
  {
    return;
  }
  exploration_path_ns::ExplorationPath path_to_follow = local_path;
  if (!follow_from_start)
  {
    path_to_follow.Reverse();
  }

  int path_node_num = path_to_follow.GetNodeNum();
  int viewpoint_count = path_to_follow.GetNodeCount(exploration_path_ns::NodeType::LOCAL_VIEWPOINT);
  if (viewpoint_count == 0)
  {
    return;
  }

  int robot_node_index = 0;
  bool has_robot_node = path_to_follow.HasNodeType(exploration_path_ns::NodeType::ROBOT, robot_node_index);
  if (!has_robot_node)
  {
    return;
  }

  int prev_cell_id = -1;
  for (int i = robot_node_index; i < path_node_num; i++)
  {
    Eigen::Vector3d node_position = path_to_follow.nodes_[i].position_;
    int cell_id = GetCellInd(node_position.x(), node_position.y(), node_position.z());
    if (cell_id != prev_cell_id)
    {
      local_exploring_cell_indices_.push_back(cell_id);
      prev_cell_id = cell_id;
      // TODO: tmp
      if (local_exploring_cell_indices_.size() >= 3)
      {
        break;
      }
    }
  }
}

void GridWorld::UpdateRobotsForConvoy(std::vector<tare::Robot>& robots)
{
  for (int robot_id = 0; robot_id < robots.size(); robot_id++)
  {
    if (robots[robot_id].id_ == kRobotID)
    {
      continue;
    }
    else if (robots[robot_id].in_comms_)
    {
      visited_for_convoy_[robot_id] = false;
    }
  }

  if (convoy_robot_id_ == kRobotID)
  {
    return;
  }

  if (robots[convoy_robot_id_].in_comms_)
  {
    return;
  }

  geometry_msgs::Point convoy_robot_position = robots[convoy_robot_id_].in_comms_position_;
  double xy_dist_to_robot =
      misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(convoy_robot_position, robot_position_);
  double z_dist_to_robot = std::abs(convoy_robot_position.z - robot_position_.z);

  if (xy_dist_to_robot < kCommsRange && z_dist_to_robot < kCellHeight)
  {
    robots[convoy_robot_id_].visited_for_convoy_ = true;
    visited_for_convoy_[convoy_robot_id_] = true;
  }
}

void GridWorld::UpdateLocalCellVisitedForRelayComms()
{
  if (!relay_comms_)
  {
    return;
  }
  MY_ASSERT(!relay_comms_robot_ids_.empty())

  for (const auto& cell_id : neighbor_cell_indices_)
  {
    geometry_msgs::Point cell_position = GetCellPosition(cell_id);
    double xy_dist_to_robot =
        misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(cell_position, robot_position_);
    double z_dist_to_robot = std::abs(cell_position.z - robot_position_.z);

    if (!GetCellViewPointIndices(cell_id).empty() ||
        xy_dist_to_robot < kCellSize && z_dist_to_robot < kCellHeight * 0.8)
    {
      for (const auto& robot_id : relay_comms_robot_ids_)
      {
        SetCellVisitedForRelayComms(cell_id, robot_id);
      }
    }
  }
}

void GridWorld::AddRoadmapNode(int cell_ind, const geometry_msgs::Point& cell_position,
                               tare::MobilityType cell_mobility_type)
{
  roadmap_.AddNode(
      RoadmapNodeType(cell_ind, cell_position, tare::Traversable(self_mobility_type_, cell_mobility_type)));
}
void GridWorld::AddRoadmapTwoWayEdge(int from_cell_ind, int to_cell_ind)
{
  roadmap_.AddEdge(from_cell_ind, to_cell_ind);
}

void GridWorld::GetExploringCellIDsAndPositions(const std::vector<tare::Robot>& robots,
                                                const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  global_exploring_cell_indices_.clear();
  local_exploring_cell_indices_.clear();

  // TODO: exclude those that are in other robots' local_exploring_cell_ids
  std::vector<int> peer_robot_local_exploring_cell_ids;
  GetPeerRobotExploringCellIDs(robots, peer_robot_local_exploring_cell_ids);

  int cell_num = subspaces_->GetCellNumber();
  for (int i = 0; i < cell_num; i++)
  {
    CellStatus cell_status = GetCellStatus(i);
    if (cell_status == CellStatus::EXPLORING)
    {
      if (!IsNeighbor(i) ||
          (subspaces_->GetCell(i).GetViewPointIndices().empty() && subspaces_->GetCell(i).GetVisitCount() > 1))
      {
        if (misc_utils_ns::ElementExistsInVector<int>(peer_robot_local_exploring_cell_ids, i))
        {
          continue;
        }
        geometry_msgs::Point cell_node;
        if (IsConnectedOnKeyposeGraph(i, keypose_graph, cell_node))
        {
          global_exploring_cell_indices_.push_back(i);
        }
      }
      else
      {
        local_exploring_cell_indices_.push_back(i);
      }
    }
    else if (cell_status == CellStatus::EXPLORING_BY_OTHERS)
    {
      if ((!IsNeighbor(i) && IsConnectedOnRoadmap(i)) ||
          (IsNeighbor(i) && IsCellTraversable(i) && subspaces_->GetCell(i).GetViewPointIndices().empty() &&
           subspaces_->GetCell(i).GetVisitCount() > 1) ||
          (IsNeighbor(i) && !IsCellTraversable(i)))
      {
        if (misc_utils_ns::ElementExistsInVector<int>(peer_robot_local_exploring_cell_ids, i))
        {
          continue;
        }
        global_exploring_cell_indices_.push_back(i);
      }
      else
      {
        local_exploring_cell_indices_.push_back(i);
      }
    }
  }
}

void GridWorld::GetDistanceMatricesWithTraversability(
    const std::vector<int>& exploring_cell_ids, const std::vector<tare::Robot>& robots,
    const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph, DistanceMatrix& distance_matrices)
{
  /* The distance matrix has the structure of
          robot1, robot2, ... , cell1, cell2, ...
   robot1   0      Inf
   robot2   Inf     0
   ...                    0
   cell1                         0
   cell2                                0
   ...
  */
  // TODO: no need for exploring cell positions

  int robot_num = robots.size();
  int cell_num = exploring_cell_ids.size();
  int distance_matrix_dim = robot_num + cell_num;

  std::stringstream robot_cell_ids_stream;
  for (int robot_index = 0; robot_index < robot_num; robot_index++)
  {
    robot_cell_ids_stream << std::to_string(GetCellInd(robots[robot_index].in_comms_position_)) << " ";
  }
  // ROS_INFO_STREAM("Robot cells: " << robot_cell_ids_stream.str());

  cell_id_to_dm_index_.clear();
  for (int i = 0; i < exploring_cell_ids.size(); i++)
  {
    cell_id_to_dm_index_[exploring_cell_ids[i]] = robot_num + i;
  }

  distance_matrices.clear();
  distance_matrices.resize(
      robot_num, std::vector<std::vector<int>>(distance_matrix_dim,
                                               std::vector<int>(distance_matrix_dim, misc_utils_ns::INF_DISTANCE)));

  // std::vector<std::vector<int>> base_distance_matrix;
  // base_distance_matrix.resize(distance_matrix_dim, std::vector<int>(distance_matrix_dim, 0));
  // MY_ASSERT(CheckDistanceMatrixSize(base_distance_matrix, distance_matrix_dim));

  // Fill the lower half of the ditance matrix
  // Robot to robot
  for (int robot_index = 0; robot_index < robot_num; robot_index++)
  {
    tare::MobilityType robot_type = robots[robot_index].mobility_type_;
    int robot_id = robots[robot_index].id_;
    bool robot_in_comms = robots[robot_index].in_comms_;
    UpdateRoadmapNodeTraversability(robot_type);
    for (int row_id = 0; row_id < robot_num; row_id++)
    {
      for (int col_id = 0; col_id < row_id; col_id++)
      {
        int from_cell_id = GetCellInd(robots[row_id].in_comms_position_);
        int to_cell_id = GetCellInd(robots[col_id].in_comms_position_);
        bool from_cell_traversable = tare::Traversable(robot_type, GetCellMobilityType(from_cell_id));
        bool to_cell_traversable = tare::Traversable(robot_type, GetCellMobilityType(to_cell_id));
        if (from_cell_traversable && to_cell_traversable)
        {
          bool computed = false;
          for (int prev_robot_index = 0; prev_robot_index < robot_index; prev_robot_index++)
          {
            if (robots[prev_robot_index].mobility_type_ == robots[robot_index].mobility_type_)
            {
              distance_matrices[robot_index][row_id][col_id] = distance_matrices[prev_robot_index][row_id][col_id];
              computed = true;
              break;
            }
          }
          if (!computed)
          {
            distance_matrices[robot_index][row_id][col_id] =
                CellToCellDistance(from_cell_id, to_cell_id, keypose_graph);
          }
        }
      }
    }
    // Cell to cell
    for (int row_id = robot_num; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = robot_num; col_id < row_id; col_id++)
      {
        int from_cell_id = exploring_cell_ids[row_id - robot_num];
        int to_cell_id = exploring_cell_ids[col_id - robot_num];
        bool from_cell_traversable = tare::Traversable(robot_type, GetCellMobilityType(from_cell_id));
        bool to_cell_traversable = tare::Traversable(robot_type, GetCellMobilityType(to_cell_id));
        if (from_cell_traversable && to_cell_traversable)
        {
          bool computed = false;
          for (int prev_robot_index = 0; prev_robot_index < robot_index; prev_robot_index++)
          {
            if (robots[prev_robot_index].mobility_type_ == robots[robot_index].mobility_type_)
            {
              distance_matrices[robot_index][row_id][col_id] = distance_matrices[prev_robot_index][row_id][col_id];
              computed = true;
              break;
            }
          }
          if (!computed)
          {
            distance_matrices[robot_index][row_id][col_id] =
                CellToCellDistance(from_cell_id, to_cell_id, keypose_graph) + kCellToCellDistanceOffset;
          }
        }
      }
    }
    // Cell to robot
    for (int row_id = robot_num; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = 0; col_id < robot_num; col_id++)
      {
        int robot_cell_id = GetCellInd(robots[col_id].in_comms_position_);
        int to_cell_id = exploring_cell_ids[row_id - robot_num];
        bool to_cell_traversable = tare::Traversable(robot_type, GetCellMobilityType(to_cell_id));
        if (to_cell_traversable)
        {
          bool computed = false;
          for (int prev_robot_index = 0; prev_robot_index < robot_index; prev_robot_index++)
          {
            if (robots[prev_robot_index].mobility_type_ == robots[robot_index].mobility_type_)
            {
              distance_matrices[robot_index][row_id][col_id] = distance_matrices[prev_robot_index][row_id][col_id];
              computed = true;
              break;
            }
          }
          if (!computed)
          {
            distance_matrices[robot_index][row_id][col_id] =
                CellToCellDistance(robot_cell_id, to_cell_id, keypose_graph);
          }
        }
      }
    }
    // Diagonal
    for (int row_id = 0; row_id < distance_matrix_dim; row_id++)
    {
      int col_id = row_id;
      distance_matrices[robot_index][row_id][col_id] = 0;
    }
    // Reflect
    for (int row_id = 0; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = row_id + 1; col_id < distance_matrix_dim; col_id++)
      {
        distance_matrices[robot_index][row_id][col_id] = distance_matrices[robot_index][col_id][row_id];
      }
    }
  }
}

void GridWorld::GetDistanceMatricesNoComms(const std::vector<int>& exploring_cell_ids,
                                           const std::vector<tare::Robot>& robots,
                                           const DistanceMatrix& distance_matrices_with_traversability,
                                           DistanceMatrix& distance_matrices_no_comms)
{
  distance_matrices_no_comms.clear();
  int robot_num = robots.size();
  int cell_num = exploring_cell_ids.size();
  int distance_matrix_dim = robot_num + cell_num;
  distance_matrices_no_comms.resize(
      robot_num, std::vector<std::vector<int>>(distance_matrix_dim,
                                               std::vector<int>(distance_matrix_dim, misc_utils_ns::INF_DISTANCE)));

  for (int robot_index = 0; robot_index < robot_num; robot_index++)
  {
    tare::MobilityType robot_type = robots[robot_index].mobility_type_;
    int robot_id = robots[robot_index].id_;
    bool robot_in_comms = robots[robot_index].in_comms_;
    for (int row_id = 0; row_id < robot_num; row_id++)
    {
      for (int col_id = 0; col_id < row_id; col_id++)
      {
        distance_matrices_no_comms[robot_index][row_id][col_id] =
            distance_matrices_with_traversability[robot_index][row_id][col_id];
      }
    }
    // Cell to cell
    for (int row_id = robot_num; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = robot_num; col_id < row_id; col_id++)
      {
        int from_cell_id = exploring_cell_ids[row_id - robot_num];
        int to_cell_id = exploring_cell_ids[col_id - robot_num];
        bool from_cell_synced = IsCellSyncedByRobot(from_cell_id, robot_id);
        bool to_cell_synced = IsCellSyncedByRobot(to_cell_id, robot_id);
        if ((from_cell_synced && to_cell_synced) || robot_in_comms)
        {
          distance_matrices_no_comms[robot_index][row_id][col_id] =
              distance_matrices_with_traversability[robot_index][row_id][col_id];
        }
      }
    }
    // Cell to robot
    for (int row_id = robot_num; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = 0; col_id < robot_num; col_id++)
      {
        int to_cell_id = exploring_cell_ids[row_id - robot_num];
        bool to_cell_synced = IsCellSyncedByRobot(to_cell_id, robot_id);
        if (to_cell_synced || robot_in_comms)
        {
          distance_matrices_no_comms[robot_index][row_id][col_id] =
              distance_matrices_with_traversability[robot_index][row_id][col_id];
        }
      }
    }
    // Diagonal
    for (int row_id = 0; row_id < distance_matrix_dim; row_id++)
    {
      int col_id = row_id;
      distance_matrices_no_comms[robot_index][row_id][col_id] = 0;
    }
    // Reflect
    for (int row_id = 0; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = row_id + 1; col_id < distance_matrix_dim; col_id++)
      {
        distance_matrices_no_comms[robot_index][row_id][col_id] =
            distance_matrices_no_comms[robot_index][col_id][row_id];
      }
    }
  }
}

void GridWorld::GetDistanceMatricesAssumeComms(const std::vector<int>& exploring_cell_ids,
                                               const std::vector<tare::Robot>& robots,
                                               const DistanceMatrix& distance_matrices_with_traversability,
                                               DistanceMatrix& distance_matrices_assume_comms)
{
  distance_matrices_assume_comms.clear();
  int robot_num = robots.size();
  int cell_num = exploring_cell_ids.size();
  int distance_matrix_dim = robot_num + cell_num;
  distance_matrices_assume_comms.resize(
      robot_num, std::vector<std::vector<int>>(distance_matrix_dim,
                                               std::vector<int>(distance_matrix_dim, misc_utils_ns::INF_DISTANCE)));

  for (int robot_index = 0; robot_index < robot_num; robot_index++)
  {
    tare::MobilityType robot_type = robots[robot_index].mobility_type_;
    int robot_id = robots[robot_index].id_;
    bool robot_in_comms = robots[robot_index].in_comms_;
    for (int row_id = 0; row_id < robot_num; row_id++)
    {
      for (int col_id = 0; col_id < row_id; col_id++)
      {
        distance_matrices_assume_comms[robot_index][row_id][col_id] =
            distance_matrices_with_traversability[robot_index][row_id][col_id];
      }
    }
    // Cell to cell
    for (int row_id = robot_num; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = robot_num; col_id < row_id; col_id++)
      {
        int from_cell_id = exploring_cell_ids[row_id - robot_num];
        int to_cell_id = exploring_cell_ids[col_id - robot_num];
        if (robot_in_comms)
        {
          distance_matrices_assume_comms[robot_index][row_id][col_id] =
              distance_matrices_with_traversability[robot_index][row_id][col_id];
        }
      }
    }
    // Cell to robot
    for (int row_id = robot_num; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = 0; col_id < robot_num; col_id++)
      {
        int to_cell_id = exploring_cell_ids[row_id - robot_num];
        if (robot_in_comms)
        {
          distance_matrices_assume_comms[robot_index][row_id][col_id] =
              distance_matrices_with_traversability[robot_index][row_id][col_id];
        }
      }
    }
    // Diagonal
    for (int row_id = 0; row_id < distance_matrix_dim; row_id++)
    {
      int col_id = row_id;
      distance_matrices_assume_comms[robot_index][row_id][col_id] = 0;
    }
    // Reflect
    for (int row_id = 0; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = row_id + 1; col_id < distance_matrix_dim; col_id++)
      {
        distance_matrices_assume_comms[robot_index][row_id][col_id] =
            distance_matrices_assume_comms[robot_index][col_id][row_id];
      }
    }
  }
}

void GridWorld::GetDistanceMatrixForCommsRelay(const DistanceMatrix& distance_matrices_with_traversability,
                                               const std::vector<tare::Robot>& in_comms_robots,
                                               const std::vector<int>& sampled_node_indices,
                                               DistanceMatrix& distance_matrices_comms_relay)
{
  std::vector<int> in_comms_robot_ids;
  for (int i = 0; i < in_comms_robots.size(); i++)
  {
    in_comms_robot_ids.push_back(in_comms_robots[i].id_);
  }
  int in_comms_robot_num = in_comms_robot_ids.size();
  int robot_num = distance_matrices_with_traversability.size();
  int cell_num = sampled_node_indices.size();
  int distance_matrix_dim = in_comms_robot_num + cell_num;

  distance_matrices_comms_relay.clear();
  distance_matrices_comms_relay.resize(
      in_comms_robot_num, std::vector<std::vector<int>>(
                              distance_matrix_dim, std::vector<int>(distance_matrix_dim, misc_utils_ns::INF_DISTANCE)));

  for (int robot_index = 0; robot_index < in_comms_robot_num; robot_index++)
  {
    int dm_robot_index = in_comms_robot_ids[robot_index];
    // Robot to robot
    for (int row_id = 0; row_id < in_comms_robot_num; row_id++)
    {
      for (int col_id = 0; col_id < row_id; col_id++)
      {
        int dm_from_robot_index = in_comms_robot_ids[row_id];
        int dm_to_robot_index = in_comms_robot_ids[col_id];
        distance_matrices_comms_relay[robot_index][row_id][col_id] =
            distance_matrices_with_traversability[dm_robot_index][dm_from_robot_index][dm_to_robot_index];
      }
    }
    // Cell to cell
    for (int row_id = in_comms_robot_num; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = in_comms_robot_num; col_id < row_id; col_id++)
      {
        int dm_from_cell_index = sampled_node_indices[row_id - in_comms_robot_num];
        int dm_to_cell_index = sampled_node_indices[col_id - in_comms_robot_num];
        distance_matrices_comms_relay[robot_index][row_id][col_id] =
            distance_matrices_with_traversability[dm_robot_index][dm_from_cell_index][dm_to_cell_index];

        if (dm_from_cell_index >= robot_num && dm_to_cell_index >= robot_num)
        {
          MY_ASSERT(distance_matrices_comms_relay[robot_index][row_id][col_id] > kCellToCellDistanceOffset)
          // They are all exploring cells rather than robot cells
          distance_matrices_comms_relay[robot_index][row_id][col_id] -= kCellToCellDistanceOffset;
        }
      }
    }
    // Cell to robot
    for (int row_id = in_comms_robot_num; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = 0; col_id < in_comms_robot_num; col_id++)
      {
        int dm_from_cell_index = sampled_node_indices[row_id - in_comms_robot_num];
        int dm_to_robot_index = in_comms_robot_ids[col_id];
        distance_matrices_comms_relay[robot_index][row_id][col_id] =
            distance_matrices_with_traversability[dm_robot_index][dm_from_cell_index][dm_to_robot_index];
      }
    }
    // Diagonal
    for (int row_id = 0; row_id < distance_matrix_dim; row_id++)
    {
      int col_id = row_id;
      distance_matrices_comms_relay[robot_index][row_id][col_id] = 0;
    }
    // Reflect
    for (int row_id = 0; row_id < distance_matrix_dim; row_id++)
    {
      for (int col_id = row_id + 1; col_id < distance_matrix_dim; col_id++)
      {
        distance_matrices_comms_relay[robot_index][row_id][col_id] =
            distance_matrices_comms_relay[robot_index][col_id][row_id];
      }
    }
  }
}

void GridWorld::GetInitialRoutes(const std::vector<tare::Robot>& robots, const std::vector<int>& exploring_cell_ids,
                                 const DistanceMatrix& distance_matrices, RouteInDMIndices& initial_routes)
{
  // std::cout << "##### Getting initial routes #####" << std::endl;
  initial_routes.clear();
  // Choose the one from other robots that has the lowest cost
  tare::VRPCost min_vrp_cost(INT_MAX, misc_utils_ns::INF_DISTANCE);
  int min_cost_robot_index = 0;
  RouteInDMIndices min_cost_route;
  for (int i = 0; i < robots.size(); i++)
  {
    if (!robots[i].in_comms_)
    {
      continue;
    }
    tare::VRPCost vrp_cost =
        GetInitialRoute(robots[i].vrp_plan_, exploring_cell_ids, distance_matrices, min_cost_route);
    if (LessCost(vrp_cost, min_vrp_cost))
    {
      min_vrp_cost = vrp_cost;
      min_cost_robot_index = i;
      initial_routes = min_cost_route;
    }
  }
}

void GridWorld::GetInitialRoutesFromVRPSolution(const std::vector<int>& exploring_cell_ids,
                                                const DistanceMatrix& distance_matrices,
                                                const RouteInDMIndices& vrp_solution, RouteInDMIndices& initial_routes)
{
  int cell_num = exploring_cell_ids.size();
  int robot_num = distance_matrices.size();
  std::vector<bool> assigned(cell_num, false);

  for (int i = 0; i < vrp_solution.size(); i++)
  {
    if (vrp_solution[i].size() > 2)
    {
      for (int j = 1; j < vrp_solution[i].size() - 1; j++)
      {
        assigned[vrp_solution[i][j] - robot_num] = true;
      }
    }
  }

  RouteInDMIndices augmented_vrp_solution = vrp_solution;
  for (int i = 0; i < assigned.size(); i++)
  {
    if (!assigned[i])
    {
      AddNodeToVRPPlan(i + robot_num, distance_matrices, augmented_vrp_solution);
    }
  }

  initial_routes.clear();
  for (int i = 0; i < augmented_vrp_solution.size(); i++)
  {
    std::vector<int> route;
    if (augmented_vrp_solution[i].size() > 2)
    {
      for (int j = 1; j < augmented_vrp_solution[i].size() - 1; j++)
      {
        route.push_back(augmented_vrp_solution[i][j]);
      }
    }
    initial_routes.push_back(route);
  }
}

void GridWorld::ConvertInitialRouteToVRPPlan(const RouteInDMIndices& initial_route, RouteInDMIndices& vrp_plan)
{
  vrp_plan.clear();
  for (int i = 0; i < initial_route.size(); i++)
  {
    std::vector<int> route;
    route.push_back(i);
    for (int j = 0; j < initial_route[i].size(); j++)
    {
      route.push_back(initial_route[i][j]);
    }
    route.push_back(i);
    vrp_plan.push_back(route);
  }
}

int GridWorld::ComputeVRPSolutionCost(const RouteInDMIndices& vrp_solution, const DistanceMatrix& distance_matrices,
                                      int& dropped_node_num, int& max_route_cost, std::vector<int>& per_route_cost,
                                      bool no_cell_to_cell_distance_offset)
{
  if (vrp_solution.empty() || distance_matrices.empty())
  {
    return 0;
  }
  MY_ASSERT(vrp_solution.size() == distance_matrices.size());
  bool is_initial_guess = false;
  for (int i = 0; i < vrp_solution.size(); i++)
  {
    if (vrp_solution[i].empty())
    {
      is_initial_guess = true;
      break;
    }
    else if (vrp_solution[i][0] != i)
    {
      is_initial_guess = true;
      break;
    }
  }
  RouteInDMIndices routes = vrp_solution;
  if (is_initial_guess)
  {
    ConvertInitialRouteToVRPPlan(vrp_solution, routes);
  }
  per_route_cost.resize(routes.size(), 0);
  int robot_num = distance_matrices.size();
  for (int i = 0; i < routes.size(); i++)
  {
    for (int j = 0; j < routes[i].size() - 1; j++)
    {
      int from_node_index = routes[i][j];
      int to_node_index = routes[i][j + 1];
      int cost = distance_matrices[i][from_node_index][to_node_index];
      if (no_cell_to_cell_distance_offset && from_node_index >= robot_num && to_node_index >= robot_num)
      {
        MY_ASSERT(cost > kCellToCellDistanceOffset)
        // They are all exploring cells rather than robot cells
        cost -= kCellToCellDistanceOffset;
      }
      per_route_cost[i] += cost;
    }
  }
  int total_node_num = distance_matrices[0].size();
  std::vector<bool> assigned(total_node_num, false);
  for (int i = 0; i < routes.size(); i++)
  {
    for (int j = 0; j < routes[i].size(); j++)
    {
      assigned[routes[i][j]] = true;
    }
  }

  dropped_node_num = 0;
  for (int i = 0; i < assigned.size(); i++)
  {
    if (!assigned[i])
    {
      dropped_node_num++;
    }
  }

  max_route_cost = 0;
  for (int i = 0; i < per_route_cost.size(); i++)
  {
    if (per_route_cost[i] > max_route_cost)
    {
      max_route_cost = per_route_cost[i];
    }
  }

  return max_route_cost * 100 + max_route_cost + dropped_node_num * misc_utils_ns::INF_DISTANCE / 10;
}

tare::VRPCost GridWorld::CombineCommsRelayCost(const RouteInDMIndices& relay_comms_vrp,
                                               const RouteInDMIndices& assume_comms_vrp,
                                               const DistanceMatrix& distance_matrices, bool print)
{
  MY_ASSERT(relay_comms_vrp.size() == assume_comms_vrp.size());

  int relay_comms_dropped_node_num = 0;
  int relay_comms_max_route_length = 0;
  std::vector<int> relay_comms_per_route_length;
  int relay_comms_total_cost = ComputeVRPSolutionCost(relay_comms_vrp, distance_matrices, relay_comms_dropped_node_num,
                                                      relay_comms_max_route_length, relay_comms_per_route_length, true);

  int assume_comms_dropped_node_num = 0;
  int assume_comms_max_route_length = 0;
  std::vector<int> assume_comms_per_route_length;
  int assume_comms_total_cost =
      ComputeVRPSolutionCost(assume_comms_vrp, distance_matrices, assume_comms_dropped_node_num,
                             assume_comms_max_route_length, assume_comms_per_route_length);

  MY_ASSERT(relay_comms_per_route_length.size() == assume_comms_per_route_length.size());
  int max_route_length = 0;
  for (int i = 0; i < relay_comms_per_route_length.size(); i++)
  {
    int route_length = relay_comms_per_route_length[i] + assume_comms_per_route_length[i];
    if (max_route_length < route_length)
    {
      max_route_length = route_length;
    }
  }

  tare::VRPCost cost;
  cost.dropped_nodes_number_ = assume_comms_dropped_node_num;
  cost.longest_route_length_ = max_route_length;
  return cost;
}

exploration_path_ns::ExplorationPath GridWorld::SolveGlobalVRP(
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager, std::vector<tare::Robot>& robots,
    std::vector<int>& ordered_cell_indices, const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  for (int i = 0; i < robots.size(); i++)
  {
    if (robots[i].id_ != i)
    {
      ROS_ERROR_STREAM("robot " << i << " id not equal to " << i);
      exit(1);
    }
  }

  geometry_msgs::Point global_path_robot_position = GetGlobalPathRobotPosition(viewpoint_manager, keypose_graph);

  // Get all exploring cells
  std::vector<int> previous_global_exploring_cell_indices = global_exploring_cell_indices_;
  GetExploringCellIDsAndPositions(robots, keypose_graph);

  // Has exploring cells last frame but not this frame, the robot has nowhere to go globally
  if (kRendezvous && !previous_global_exploring_cell_indices.empty() && global_exploring_cell_indices_.empty())
  {
    go_to_rendezvous_ = true;
  }

  if (global_exploring_cell_indices_.empty() || (kUseTimeBudget && time_budget_manager_.TimesUp()))
  {
    no_comms_global_plan_.clear();
    relay_comms_global_plan_.clear();
    assume_comms_global_plan_.clear();
    return_home_ = true;
    relay_comms_ = false;
    is_sampled_last_iteration_.resize(kRobotNum, false);

    // If all robots are in comms, don't go to rendezvous
    if (kRendezvous && go_to_rendezvous_)
    {
      if (AllRobotsInComms(robots))
      {
        go_to_rendezvous_ = false;
        wait_ = false;
      }
      else
      {
        int robot_cell_id = GetCellInd(robot_position_);
        int rendezvous_cell_id = rendezvous_manager_.GetCurrentRendezvousCellID();
        if (!ConnectedToRendezvous(robots, rendezvous_cell_id))
        {
          wait_ = false;
          rendezvous_global_plan_.clear();
          rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
          rendezvous_global_plan_.push_back(rendezvous_cell_id);
          rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
        }
        else
        {
          wait_ = true;
          rendezvous_global_plan_.clear();
          rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
          rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
        }
        return GetGlobalExplorationPath(rendezvous_global_plan_, keypose_graph);
      }
    }

    return GetReturnHomePath(global_path_robot_position, keypose_graph);
  }

  return_home_ = false;

  // Set self vrp plan from last
  robots[kRobotID].vrp_plan_.cost_ = vrp_cost_;
  robots[kRobotID].vrp_plan_.sampled_robots_ = SampleRobotVectorToInt(is_sampled_last_iteration_);
  robots[kRobotID].vrp_plan_.no_comms_ordered_cell_ids_ = no_comms_global_plan_;

  if (kRendezvous)
  {
    robots[kRobotID].vrp_plan_.relay_comms_ordered_cell_ids_.clear();
    std::vector<int> tmp;
    tmp.push_back(rendezvous_manager_.GetNextRendezvousCellID());
    tmp.push_back(rendezvous_manager_.GetNextRendezvousTimeInterval());
    robots[kRobotID].vrp_plan_.relay_comms_ordered_cell_ids_.push_back(tmp);
  }
  else
  {
    robots[kRobotID].vrp_plan_.relay_comms_ordered_cell_ids_ = relay_comms_global_plan_;
  }
  robots[kRobotID].vrp_plan_.assume_comms_ordered_cell_ids_ = assume_comms_global_plan_;
  robots[kRobotID].vrp_plan_.local_exploring_cell_ids_ = local_exploring_cell_indices_;

  CheckLostRobot(robots);  // If a robot is lost, the others will not try to find it

  //////////////////////Start getting distance matrix//////////////////////
  misc_utils_ns::Timer timer1("Get distance matrix");
  timer1.Start();
  // Construct the distance matrix with keypose_graph and roadmap_ and initial routes from last planned routes
  if (kUseTimeBudget)
  {
    kCellToCellDistanceOffset = 0;
  }
  DistanceMatrix distance_matrices_with_traversability;
  GetDistanceMatricesWithTraversability(global_exploring_cell_indices_, robots, keypose_graph,
                                        distance_matrices_with_traversability);
  timer1.Stop(false);
  int timer1_time = timer1.GetDuration();

  misc_utils_ns::Timer timer11("Get distance matrix no comms");
  timer11.Start();
  DistanceMatrix distance_matrices_no_comms;
  GetDistanceMatricesNoComms(global_exploring_cell_indices_, robots, distance_matrices_with_traversability,
                             distance_matrices_no_comms);
  timer11.Stop(false);
  int timer11_time = timer11.GetDuration();
  //////////////////////End getting distance matrix//////////////////////

  misc_utils_ns::Timer timer12("Get Initial routes");
  timer12.Start();
  RouteInDMIndices initial_routes;
  GetInitialRoutes(robots, global_exploring_cell_indices_, distance_matrices_no_comms, initial_routes);
  timer12.Stop(false);
  int timer12_time = timer12.GetDuration();

  misc_utils_ns::Timer timer2("No comms VRP");
  timer2.Start();

  RouteInDMIndices no_comms_vrp_solution;  // Indices w.r.t. distance matrix
  tare::VRPCost no_comms_vrp_cost = SolveVRP(distance_matrices_no_comms, initial_routes, no_comms_vrp_solution);
  vrp_cost_ = no_comms_vrp_cost;

  MY_ASSERT(no_comms_vrp_solution.size() == robots.size());

  timer2.Stop(false);
  int timer2_time = timer2.GetDuration();

  misc_utils_ns::Timer timer3("Assumes comms VRP");
  timer3.Start();
  UpdateLocalCellVisitedForRelayComms();

  RouteInDMIndices assume_comms_vrp_solution;
  RouteInDMIndices relay_comms_vrp_solution;
  if (kUseTimeBudget)
  {
    // ROS_INFO_STREAM("----Using time budget----");
  }
  else if (kRendezvous)
  {
    // ROS_INFO_STREAM("----Planning for rendezvous----");
    go_to_rendezvous_ = PlanForRendezvous(robots, distance_matrices_with_traversability, rendezvous_global_plan_);
  }
  else if (kRelayComms)
  {
    if (HasKnowledgeToShare(robots))
    {
      // ROS_INFO_STREAM(misc_utils_ns::ColoredText("Has new knowledge", misc_utils_ns::TextColor::GREEN));
      // ROS_INFO_STREAM("----Solving vrp assumes comms----");
      relay_comms_ = SolveVRPAssumeComms(global_exploring_cell_indices_, robots, no_comms_vrp_cost,
                                         no_comms_vrp_solution, distance_matrices_with_traversability, keypose_graph,
                                         assume_comms_vrp_solution, relay_comms_vrp_solution);
      // ROS_INFO_STREAM("----Finished solving vrp assume comms----");
    }
    else
    {
      // ROS_INFO_STREAM(misc_utils_ns::ColoredText("Do not have new knowledge", misc_utils_ns::TextColor::RED));
      relay_comms_ = false;
    }
  }
  else
  {
    relay_comms_ = false;
  }

  timer3.Stop(false);
  int timer3_time = timer3.GetDuration();

  // std::cout << "----Final solution----" << std::endl;
  // PrintVRPSolution(vrp_solution);
  misc_utils_ns::Timer timer4("Get path");
  timer4.Start();
  // From distance matrix indexing to ordered cell ids
  VRPSolutionToOrderedCellIDs(robots, global_exploring_cell_indices_, no_comms_vrp_solution, no_comms_global_plan_);
  VRPSolutionToOrderedCellIDs(robots, global_exploring_cell_indices_, relay_comms_vrp_solution,
                              relay_comms_global_plan_);
  VRPSolutionToOrderedCellIDs(robots, global_exploring_cell_indices_, assume_comms_vrp_solution,
                              assume_comms_global_plan_);
  CheckNotVisitedCells(no_comms_global_plan_);

  ordered_cell_indices.clear();
  if (kRendezvous && go_to_rendezvous_)
  {
    ordered_cell_indices = rendezvous_global_plan_;
  }
  else
  {
    if (relay_comms_)
    {
      MY_ASSERT(!relay_comms_global_plan_[kRobotID].empty());
      ordered_cell_indices = relay_comms_global_plan_[kRobotID];
      // ROS_INFO_STREAM("relaying comms to: ");
      // misc_utils_ns::PrintVector<int>(relay_comms_robot_ids_);
      // ROS_INFO_STREAM("relay comms plan: ");
      // PrintVRPSolution(relay_comms_vrp_solution);
    }
    else
    {
      MY_ASSERT(!no_comms_global_plan_[kRobotID].empty());
      ordered_cell_indices = no_comms_global_plan_[kRobotID];
      // If there is no plan for self, go to the nearest exploring global subspace
      if (ordered_cell_indices.size() == 2 && global_exploring_cell_indices_.size() > 0 &&
          IsConnectedOnRoadmap(cur_robot_cell_ind_))
      {
        int nearest_cell_ind = -1;
        double min_dist = roadmap_.GetEdgeCutOffDistanceThreshold();
        for (const auto& cell_ind : global_exploring_cell_indices_)
        {
          if (!IsCellTraversable(cell_ind) || !IsConnectedOnRoadmap(cell_ind))
          {
            continue;
          }
          nav_msgs::Path shortest_path;
          std::vector<int> node_ids;
          double dist = roadmap_.GetShortestPath(cur_robot_cell_ind_, cell_ind, false, shortest_path, node_ids);
          if (dist < min_dist)
          {
            min_dist = dist;
            nearest_cell_ind = cell_ind;
          }
        }
        if (nearest_cell_ind != -1)
        {
          ordered_cell_indices.push_back(cur_robot_cell_ind_);
          ordered_cell_indices.push_back(nearest_cell_ind);
          ordered_cell_indices.push_back(cur_robot_cell_ind_);
        }
      }
    }
  }

  // Get the global exploration path
  exploration_path_ns::ExplorationPath global_path;
  if (ordered_cell_indices.size() == 2 && !wait_)
  {
    global_path = GetReturnHomePath(global_path_robot_position, keypose_graph);
  }
  else
  {
    global_path = GetGlobalExplorationPath(ordered_cell_indices, keypose_graph);
  }
  timer4.Stop(false);
  int timer4_time = timer4.GetDuration();

  // ROS_INFO_STREAM("global planning takes: dm(" << timer1_time << ") init(" << timer12_time << ") no-comms-VRP("
  //                                              << timer2_time << ") assume-comms-vrp(" << timer3_time << ")
  //                                              get-path("
  //                                              << timer4_time << ")");
  return global_path;
}

exploration_path_ns::ExplorationPath GridWorld::PlanGlobalConvoy(
    std::vector<tare::Robot>& robots, const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  UpdateRobotsForConvoy(robots);

  int nearest_robot_id = kRobotID;
  int shortest_distance = roadmap_.GetEdgeCutOffDistanceThreshold();
  int from_cell_id = GetCellInd(robot_position_);
  if (!IsConnectedOnRoadmap(from_cell_id))
  {
    exploration_path_ns::ExplorationPath empty_path;
    return empty_path;
  }
  // Follow those thare are in comms first
  for (int robot_id = 0; robot_id < robots.size(); robot_id++)
  {
    if (robot_id != kRobotID && !robots[robot_id].vrp_plan_.local_exploring_cell_ids_.empty() &&
        robots[robot_id].in_comms_)
    {
      int to_cell_id = GetCellInd(robots[robot_id].in_comms_position_);
      if (!IsConnectedOnRoadmap(to_cell_id))
      {
        continue;
      }
      nav_msgs::Path shortest_path;
      std::vector<int> node_ids;
      int distance_to_robot = roadmap_.GetShortestPath(from_cell_id, to_cell_id, false, shortest_path, node_ids);
      if (distance_to_robot < shortest_distance)
      {
        shortest_distance = distance_to_robot;
        nearest_robot_id = robot_id;
      }
    }
  }
  if (nearest_robot_id == kRobotID)
  {
    // Follow those that are out of comms
    shortest_distance = roadmap_.GetEdgeCutOffDistanceThreshold();
    for (int robot_id = 0; robot_id < robots.size(); robot_id++)
    {
      if (robot_id != kRobotID && !robots[robot_id].finished_exploration_ && !visited_for_convoy_[robot_id])
      {
        int to_cell_id = GetCellInd(robots[robot_id].in_comms_position_);
        if (!IsConnectedOnRoadmap(to_cell_id))
        {
          continue;
        }
        nav_msgs::Path shortest_path;
        std::vector<int> node_ids;
        int distance_to_robot = roadmap_.GetShortestPath(from_cell_id, to_cell_id, false, shortest_path, node_ids);
        if (distance_to_robot < shortest_distance)
        {
          shortest_distance = distance_to_robot;
          nearest_robot_id = robot_id;
        }
      }
    }
  }

  if (nearest_robot_id == kRobotID)
  {
    exploration_path_ns::ExplorationPath empty_path;
    return empty_path;
  }

  convoy_robot_id_ = nearest_robot_id;

  int to_cell_id = GetCellInd(robots[nearest_robot_id].in_comms_position_);
  std::vector<int> cell_ids;
  cell_ids.push_back(from_cell_id);
  cell_ids.push_back(to_cell_id);
  cell_ids.push_back(from_cell_id);
  // ROS_INFO_STREAM(misc_utils_ns::ColoredText("Following robot " + std::to_string(nearest_robot_id),
  //                                            misc_utils_ns::TextColor::YELLOW));
  return GetGlobalExplorationPath(cell_ids, keypose_graph);
}

int GridWorld::GetCellKeyposeID(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetKeyposeID();
}

void GridWorld::GetCellViewPointPositions(std::vector<Eigen::Vector3d>& viewpoint_positions)
{
  viewpoint_positions.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() != grid_world_ns::CellStatus::EXPLORING)
    {
      continue;
    }
    if (!IsNeighbor(i))
    {
      viewpoint_positions.push_back(subspaces_->GetCell(i).GetViewPointPosition());
    }
  }
}

/**
 * @brief
 * @param robot_id
 * @param ordered_cell_ids
 * @param distance_matrix
 * @param time_resolution
 * @param time_steps
 * @param sampled_cell_ids dim(N), where N is the number of sampled exploring cells,
 * @param[out] arrival_probability dim(N), where N is the number of exploring cells in the ordered_cell_ids,
 * arrival_probability[i][j] is the probability that the robot will arrive at cell i at timestep j
 */
void GridWorld::ComputeArrivalProbability(int robot_id, const std::vector<int>& ordered_cell_ids,
                                          const DistanceMatrix& distance_matrix, double time_resolution, int time_steps,
                                          std::vector<int>& sampled_cell_ids,
                                          std::vector<std::vector<double>>& arrival_probability)
{
  if (ordered_cell_ids.empty())
  {
    return;
  }

  std::vector<int> exploring_ordered_cell_ids;
  // Add in the first robot cell to get the distance to the first exploring cell
  exploring_ordered_cell_ids.push_back(ordered_cell_ids[0]);

  for (const auto& cell_id : ordered_cell_ids)
  {
    if (IsCellVisitedForRelayComms(cell_id, robot_id))
    {
      continue;
    }

    if (misc_utils_ns::ElementExistsInVector<int>(global_exploring_cell_indices_, cell_id) &&
        !misc_utils_ns::ElementExistsInVector<int>(exploring_ordered_cell_ids, cell_id))
    {
      exploring_ordered_cell_ids.push_back(cell_id);
    }
  }

  if (exploring_ordered_cell_ids.size() == 1)
  {
    // Cannot compute arrival probability if there is only one cell
    return;
  }

  // Remove the first robot cell
  sampled_cell_ids.clear();
  for (int i = 1; i < exploring_ordered_cell_ids.size(); i++)
  {
    sampled_cell_ids.push_back(exploring_ordered_cell_ids[i]);
  }

  std::vector<double> distance_between_cells;
  for (int i = 0; i < exploring_ordered_cell_ids.size() - 1; i++)
  {
    int from_cell_id = exploring_ordered_cell_ids[i];
    int to_cell_id = exploring_ordered_cell_ids[i + 1];

    int from_cell_dm_index = cell_id_to_dm_index_[from_cell_id];
    int to_cell_dm_index = cell_id_to_dm_index_[to_cell_id];

    int cell_to_cell_dist = distance_matrix[robot_id][from_cell_dm_index][to_cell_dm_index];
    // Remove cell to cell distance offset
    if (from_cell_dm_index >= kRobotNum && to_cell_dm_index >= kRobotNum)
    {
      cell_to_cell_dist -= kCellToCellDistanceOffset;
    }
    MY_ASSERT(cell_to_cell_dist >= 0);
    distance_between_cells.push_back(cell_to_cell_dist);
  }

  misc_utils_ns::Timer mdp_timer("mdp");
  mdp_timer.Start();
  PursuitMDP mdp;
  std::vector<std::vector<double>> arrival_probability_t;
  mdp.EvaluateMDP(distance_between_cells, time_resolution, time_steps, arrival_probability_t);
  mdp_timer.Stop(false);

  // Transpose the 2d matrix
  arrival_probability.clear();
  for (int i = 0; i < arrival_probability_t[0].size(); i++)
  {
    std::vector<double> arrival_probability_row;
    for (int j = 0; j < arrival_probability_t.size(); j++)
    {
      arrival_probability_row.push_back(arrival_probability_t[j][i]);
    }
    arrival_probability.push_back(arrival_probability_row);
  }

  MY_ASSERT(arrival_probability.size() == sampled_cell_ids.size());

  // ROS_INFO_STREAM("robot id: " << robot_id);
  // ROS_INFO_STREAM("ordered cell ids: ");
  // misc_utils_ns::PrintVector<int>(ordered_cell_ids);
  // for (const auto& cell_id : ordered_cell_ids)
  // {
  //   std::cout << cell_id << " ";
  // }
  // std::cout << std::endl;
  // ROS_INFO_STREAM("exploring ordered cell ids: ");
  // misc_utils_ns::PrintVector<int>(exploring_ordered_cell_ids);
  // for (const auto& cell_id : exploring_ordered_cell_ids)
  // {
  //   std::cout << cell_id << " ";
  // }
  // std::cout << std::endl;
  // ROS_INFO_STREAM("distance between cells: ");
  // misc_utils_ns::PrintVector<double>(distance_between_cells);
  // for (const auto& distance : distance_between_cells)
  // {
  //   std::cout << distance << " ";
  // }
  // std::cout << std::endl;

  // ROS_INFO_STREAM("sampled cell ids: ");
  // misc_utils_ns::PrintVector<int>(sampled_cell_ids);
  // for (const auto& cell_id : sampled_cell_ids)
  // {
  //   std::cout << cell_id << " ";
  // }
  // std::cout << std::endl;
  // std::cout << "arrival probability: " << std::endl;
  // // print arrival probability to cout
  // for (int i = 0; i < arrival_probability.size(); i++)
  // {
  //   for (int j = 0; j < arrival_probability[i].size(); j++)
  //   {
  //     std::cout << arrival_probability[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }
}

void GridWorld::VisualizeArrivalProbability(
    const std::vector<int>& pursuit_cell_ids, const std::vector<std::vector<double>>& arrival_probability,
    std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>& arrival_probability_cloud)
{
  arrival_probability_cloud_->cloud_->clear();

  for (int i = 0; i < pursuit_cell_ids.size(); i++)
  {
    int cell_id = pursuit_cell_ids[i];
    geometry_msgs::Point cell_position = GetCellPosition(cell_id);
    double position_z = cell_position.z;
    for (int j = 0; j < arrival_probability[i].size(); j++)
    {
      double prob = arrival_probability[i][j];
      pcl::PointXYZI point;
      point.x = cell_position.x + prob * 10;
      point.y = cell_position.y;
      point.z = position_z + j * 1.5;
      point.intensity = prob;
      arrival_probability_cloud_->cloud_->push_back(point);
    }
  }
  arrival_probability_cloud_->Publish();
}

/// @brief Combine the arrival probability of each robot's to-visit cells
/// @param sampled_cell_ids_all_robots: robot_num x var1
/// @param arrival_probability_all_robots: robot_num x var1 x time_steps
/// @param unique_sampled_cell_ids: var3
/// @param combined_arrival_probability: var3 x time_steps
void GridWorld::CombineArrivalProbability(
    const std::vector<std::vector<int>>& sampled_cell_ids_all_robots,
    const std::vector<std::vector<std::vector<double>>>& arrival_probability_all_robots,
    std::vector<int>& unique_sampled_cell_ids, std::vector<std::vector<double>>& combined_arrival_probability)
{
  unique_sampled_cell_ids.clear();
  combined_arrival_probability.clear();

  for (int i = 0; i < sampled_cell_ids_all_robots.size(); i++)
  {
    for (int j = 0; j < sampled_cell_ids_all_robots[i].size(); j++)
    {
      int duplicate_index =
          std::find(unique_sampled_cell_ids.begin(), unique_sampled_cell_ids.end(), sampled_cell_ids_all_robots[i][j]) -
          unique_sampled_cell_ids.begin();
      if (duplicate_index == unique_sampled_cell_ids.size())
      {
        unique_sampled_cell_ids.push_back(sampled_cell_ids_all_robots[i][j]);
        combined_arrival_probability.push_back(arrival_probability_all_robots[i][j]);
      }
      else
      {
        // Combine the arrival probability
        for (int k = 0; k < combined_arrival_probability[duplicate_index].size(); k++)
        {
          combined_arrival_probability[duplicate_index][k] += arrival_probability_all_robots[i][j][k];
        }
      }
    }
  }

  // TODO: remove this
  for (int i = 0; i < combined_arrival_probability.size(); i++)
  {
    for (int j = 0; j < combined_arrival_probability[i].size(); j++)
    {
      combined_arrival_probability[i][j] = std::max(combined_arrival_probability[i][j], 0.01);
    }
  }

  MY_ASSERT(combined_arrival_probability.size() == unique_sampled_cell_ids.size());
}

/**
 * @brief
 *
 * @param distance_matrices_with_traversability
 * @param robots
 * @param is_sampled
 * @param sampled_node_indices
 * @param[out] pursuit_cell_ids dim[N]: the cell ids of the sampled nodes on the routes of out-of-comms robots
 * @param[out] arrival_probability dim[(N + 1) x T]: the arrival probability of the sampled nodes on the routes of
 * out-of-comms robots, the first row corresponds to the start node where all in-comms robots are, of which the arrival
 * probabilities(profit) are 0.
 * @param[out] pursuit_distance_matrix dim[(N + 1) x (N + 1)]: the distance matrix of the sampled nodes on the routes of
 * out-of-comms
 */
void GridWorld::ComputeArrivalProbabilityOutOfCommsRobotRoutes(
    const DistanceMatrix& distance_matrices_with_traversability, const std::vector<tare::Robot>& robots,
    const std::vector<bool>& is_sampled, double time_resolution, int max_time_steps, std::vector<int>& pursuit_cell_ids,
    std::vector<std::vector<double>>& arrival_probability)
{
  // Step 1: For each sampled robot, compute the arrival probability on its route
  std::vector<std::vector<int>> sampled_cell_ids_all_robots;
  std::vector<std::vector<std::vector<double>>> arrival_probability_all_robots;
  for (int robot_id = 0; robot_id < robots.size(); robot_id++)
  {
    if (!is_sampled[robot_id])
    {
      continue;
    }

    // ROS_INFO_STREAM("computing arrival probability for robot " << robot_id);
    if (robots[robot_id].vrp_plan_.no_comms_ordered_cell_ids_.empty())
    {
      // ROS_INFO_STREAM("robot " << robot_id << " has no no-comms plan, skipping");
      continue;
    }

    std::vector<int> sampled_cell_ids_one_robot;
    std::vector<std::vector<double>> arrival_probability_one_robot;
    // ROS_INFO_STREAM("robot " << robot_id << "vrp_plan no comms ordered cell ids_: ");
    // misc_utils_ns::PrintVector<int>(robots[robot_id].vrp_plan_.no_comms_ordered_cell_ids_[robot_id]);
    // for (const auto& cell_id : robots[robot_id].vrp_plan_.no_comms_ordered_cell_ids_[robot_id])
    // {
    //   std::cout << cell_id << " ";
    // }
    // std::cout << std::endl;
    ComputeArrivalProbability(robot_id, robots[robot_id].vrp_plan_.no_comms_ordered_cell_ids_[robot_id],
                              distance_matrices_with_traversability, time_resolution, max_time_steps,
                              sampled_cell_ids_one_robot, arrival_probability_one_robot);
    sampled_cell_ids_all_robots.push_back(sampled_cell_ids_one_robot);
    arrival_probability_all_robots.push_back(arrival_probability_one_robot);
  }
  // Step 2: Combine the arrival probability of all sampled cell
  CombineArrivalProbability(sampled_cell_ids_all_robots, arrival_probability_all_robots, pursuit_cell_ids,
                            arrival_probability);

  // Visualize the arrival probability
  VisualizeArrivalProbability(pursuit_cell_ids, arrival_probability, arrival_probability_cloud_);

  // Step 3: Compute the distance matrix of the sampled cells
  // ComputePursuitDistanceMatrix(pursuit_cell_ids, distance_matrices_with_traversability,
  // pursuit_distance_matrix);
}

void GridWorld::ComputePursuitDistanceMatrix(const std::vector<tare::Robot>& robots,
                                             const std::vector<int>& pursuit_robot_ids,
                                             const std::vector<int>& pursuit_cell_ids,
                                             const DistanceMatrix& distance_matrices_with_traversability,
                                             double time_resolution,
                                             std::vector<std::vector<int>>& pursuit_distance_matrix)
{
  // pursuit_distance_matrix has the size of (pursuit_robot_ids.size() + pursuit_cell_ids.size())
  int num_pursuit_robot = pursuit_robot_ids.size();
  int num_pursuit_cell = pursuit_cell_ids.size();
  int pursuit_dm_size = num_pursuit_robot + num_pursuit_cell;
  int self_id = kRobotID;
  pursuit_distance_matrix.resize(pursuit_dm_size, std::vector<int>(pursuit_dm_size, 0));
  for (int i = 0; i < pursuit_dm_size; i++)
  {
    for (int j = 0; j < i; j++)
    {
      if (i < num_pursuit_robot && j < num_pursuit_robot)
      {
        // The distance between two robots
        pursuit_distance_matrix[i][j] =
            distance_matrices_with_traversability[kRobotID][pursuit_robot_ids[i]][pursuit_robot_ids[j]];
      }
      else if (i < num_pursuit_robot && j >= num_pursuit_robot)
      {
        // The distance between a robot and a cell
        int cell_id = pursuit_cell_ids[j - num_pursuit_robot];
        MY_ASSERT(misc_utils_ns::ElementExistsInVector<int>(global_exploring_cell_indices_, cell_id));
        int cell_dm_index = cell_id_to_dm_index_[cell_id];
        pursuit_distance_matrix[i][j] =
            distance_matrices_with_traversability[kRobotID][pursuit_robot_ids[i]][cell_dm_index];
      }
      else if (i >= num_pursuit_robot && j < num_pursuit_robot)
      {
        // The distance between a cell and a robot
        int cell_id = pursuit_cell_ids[i - num_pursuit_robot];
        MY_ASSERT(misc_utils_ns::ElementExistsInVector<int>(global_exploring_cell_indices_, cell_id));
        int cell_dm_index = cell_id_to_dm_index_[cell_id];
        pursuit_distance_matrix[i][j] =
            distance_matrices_with_traversability[kRobotID][cell_dm_index][pursuit_robot_ids[j]];
      }
      else
      {
        // The distance between two cells
        int from_cell_id = pursuit_cell_ids[i - num_pursuit_robot];
        MY_ASSERT(misc_utils_ns::ElementExistsInVector<int>(global_exploring_cell_indices_, from_cell_id));
        int from_cell_dm_index = cell_id_to_dm_index_[from_cell_id];
        int to_cell_id = pursuit_cell_ids[j - num_pursuit_robot];
        MY_ASSERT(misc_utils_ns::ElementExistsInVector<int>(global_exploring_cell_indices_, to_cell_id));
        int to_cell_dm_index = cell_id_to_dm_index_[to_cell_id];

        pursuit_distance_matrix[i][j] =
            distance_matrices_with_traversability[kRobotID][from_cell_dm_index][to_cell_dm_index];

        // Remove cell to cell distance offset
        MY_ASSERT(from_cell_dm_index >= kRobotNum && to_cell_dm_index >= kRobotNum);
        pursuit_distance_matrix[i][j] =
            distance_matrices_with_traversability[kRobotID][from_cell_dm_index][to_cell_dm_index] -
            kCellToCellDistanceOffset;
      }
    }
  }

  // Reflect the lower triangle to the upper triangle
  for (int i = 0; i < pursuit_dm_size; i++)
  {
    for (int j = i + 1; j < pursuit_dm_size; j++)
    {
      pursuit_distance_matrix[i][j] = pursuit_distance_matrix[j][i];
    }
  }

  // Account for time resolution
  for (int i = 0; i < pursuit_dm_size; i++)
  {
    for (int j = 0; j < pursuit_dm_size; j++)
    {
      if (i == j)
      {
        continue;
      }
      pursuit_distance_matrix[i][j] = std::max(int(pursuit_distance_matrix[i][j] / time_resolution), 1);
    }
  }
}

void GridWorld::SampleNodesFromOutOfCommsRobotRoutes(const DistanceMatrix& distance_matrices_with_traversability,
                                                     const std::vector<tare::Robot>& robots,
                                                     const std::vector<bool>& is_sampled,
                                                     std::vector<std::pair<int, int>>& time_windows,
                                                     std::vector<int>& sampled_node_indices)
{
  int overall_node_num = distance_matrices_with_traversability.front().size();
  int robot_num = robots.size();
  time_windows.resize(overall_node_num, std::make_pair(0, misc_utils_ns::INF_DISTANCE));

  sampled_node_indices.clear();

  // if (no_comms_vrp_solution.empty())
  // {
  //   ROS_WARN("No comms vrp solution empty, cannot sample nodes");
  //   return;
  // }

  // MY_ASSERT(no_comms_vrp_solution.size() == robots.size());

  for (int robot_id = 0; robot_id < robots.size(); robot_id++)
  {
    if (is_sampled[robot_id])
    {
      if (robot_id == kRobotID || robots[robot_id].in_comms_ || robots[robot_id].lost_)
      {
        ROS_ERROR_STREAM("Invalid sampled robot_id: " << robot_id << " in comms: " << robots[robot_id].in_comms_
                                                      << " lost: " << robots[robot_id].lost_);
        continue;
      }

      if (!misc_utils_ns::ElementExistsInVector<int>(sampled_node_indices, robot_id))
      {
        int robot_cell_id = GetCellInd(robots[robot_id].in_comms_position_);
        if (!IsCellVisitedForRelayComms(robot_cell_id, robot_id))
        {
          sampled_node_indices.push_back(robot_id);
        }
      }
      for (int j = 0; j < global_exploring_cell_indices_.size(); j++)
      {
        int cell_id = global_exploring_cell_indices_[j];
        if (IsCellSyncedByRobot(cell_id, robot_id) && !IsCellVisitedForRelayComms(cell_id, robot_id))
        {
          int dm_index = robot_num + j;
          if (!misc_utils_ns::ElementExistsInVector<int>(sampled_node_indices, dm_index))
          {
            sampled_node_indices.push_back(dm_index);
          }
        }
      }
    }
  }
}

bool GridWorld::SampleOutOfCommsRobotIndices(const std::vector<tare::Robot>& robots, int sample_num,
                                             std::vector<std::vector<bool>>& is_sampled)
{
  is_sampled.clear();

  std::vector<int> out_of_comms_robot_indices;
  for (int i = 0; i < robots.size(); i++)
  {
    if (!robots[i].in_comms_ && !robots[i].lost_)
    {
      out_of_comms_robot_indices.push_back(i);
    }
  }
  int out_of_comms_robots_num = out_of_comms_robot_indices.size();
  // ROS_INFO_STREAM("out of comms robot number: " << out_of_comms_robots_num);

  if (out_of_comms_robot_indices.empty())
  {
    is_sampled_last_iteration_.clear();
    is_sampled_last_iteration_.resize(kRobotNum, false);
    return false;
  }

  // Add samples from the last iteration
  std::vector<int32_t> sampled_numbers;
  bool add_last_sample = false;
  for (int i = 0; i < is_sampled_last_iteration_.size(); i++)
  {
    if (!robots[i].in_comms_ && !robots[i].lost_ && is_sampled_last_iteration_[i])
    {
      add_last_sample = true;
    }
    else if ((robots[i].in_comms_ || robots[i].lost_) && is_sampled_last_iteration_[i])
    {
      is_sampled_last_iteration_[i] = false;
    }
  }

  if (add_last_sample)
  {
    int32_t sample_number_last_iteration = SampleRobotVectorToInt(is_sampled_last_iteration_);
    sampled_numbers.push_back(sample_number_last_iteration);
    is_sampled.push_back(is_sampled_last_iteration_);
    sample_num--;
    // ROS_INFO_STREAM("is sampled last iteration: ");
    // misc_utils_ns::PrintVector<bool>(is_sampled_last_iteration_);
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  int32_t min_sample = 1;
  int32_t max_sample = pow(2, out_of_comms_robots_num) - 1;
  std::uniform_int_distribution<int32_t> gen_next_sample(min_sample, max_sample);

  for (int itr = 0; itr < sample_num; itr++)
  {
    int sample = gen_next_sample(gen);
    std::vector<bool> is_sampled_itr(kRobotNum, false);
    int sampled_number = 0;
    for (int i = 0; i < out_of_comms_robots_num; i++)
    {
      if (sample & (1 << i))
      {
        int robot_id = out_of_comms_robot_indices[i];
        is_sampled_itr[robot_id] = true;
        sampled_number += (1 << robot_id);
      }
    }
    if (!misc_utils_ns::ElementExistsInVector<int32_t>(sampled_numbers, sampled_number))
    {
      is_sampled.push_back(is_sampled_itr);
      sampled_numbers.push_back(sampled_number);
    }
  }

  // ROS_INFO_STREAM("sampled numbers: ");
  // misc_utils_ns::PrintVector<int32_t>(sampled_numbers);

  return true;
}

std::pair<int, int> GridWorld::SampleNodeFromRoute(int robot_index, const std::vector<int>& robot_route,
                                                   const DistanceMatrix& distance_matrices)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  int node_num = robot_route.size();

  // std::uniform_int_distribution<int> gen_next_node_index(0, node_num - 1);
  // int node_index = gen_next_node_index(gen);
  // TODO: use randomly sampled node index above
  int node_index = node_num - 1;
  MY_ASSERT(node_index >= 0 && node_index < robot_route.size());

  int dist = 0;
  for (int i = 1; i <= node_index; i++)
  {
    int prev_index = robot_route[i - 1];
    int curr_index = robot_route[i];
    dist += distance_matrices[robot_index][prev_index][curr_index];
  }
  return std::make_pair(robot_route[node_index], dist);
}

bool GridWorld::HasFrontierViewpoints(int cell_ind,
                                      const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  std::vector<int> viewpoint_indices = subspaces_->GetCell(cell_ind).GetViewPointIndices();
  for (const auto& viewpoint_ind : viewpoint_indices)
  {
    if (viewpoint_manager->IsViewPointFrontier(viewpoint_ind))
    {
      return true;
    }
  }
  return false;
}

bool GridWorld::HasKeyposeNodes(int cell_ind, const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  std::vector<int> keypose_graph_node_indices = subspaces_->GetCell(cell_ind).GetGraphNodeIndices();
  for (const auto& node_ind : keypose_graph_node_indices)
  {
    if (keypose_graph->IsNodeKeypose(node_ind))
    {
      return true;
    }
  }
  return false;
}

void GridWorld::SetRoadmapConnectionPointForCells(
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  for (int i = 0; i < neighbor_cell_indices_.size(); i++)
  {
    int cell_ind = neighbor_cell_indices_[i];
    bool cell_has_keypose_node = HasKeyposeNodes(cell_ind, keypose_graph);
    if (subspaces_->GetCell(cell_ind).IsRoadmapConnectionPointSet())
    {
      Eigen::Vector3d roadmap_connection_point = subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint();
      bool in_collision = viewpoint_manager->InLocalPlanningHorizon(roadmap_connection_point) &&
                          viewpoint_manager->InCollision(roadmap_connection_point);

      bool connection_point_is_keypose = keypose_graph->IsNodeKeypose(
          keypose_graph->GetClosestNodeInd(misc_utils_ns::Eigen2GeoMsgPnt(roadmap_connection_point)));

      if (in_collision || (cell_has_keypose_node && !connection_point_is_keypose))
      {
        // Reset the connection point
      }
      else
      {
        continue;
      }
    }

    geometry_msgs::Point cell_center_position = subspaces_->GetCell(cell_ind).GetPosition();
    std::vector<int> keypose_graph_node_indices = subspaces_->GetCell(cell_ind).GetGraphNodeIndices();

    bool connection_point_set = false;
    if (!keypose_graph_node_indices.empty())
    {
      double min_dist = DBL_MAX;
      int min_dist_keypose_graph_node_ind = keypose_graph_node_indices.front();
      for (const auto& keypose_graph_node_ind : keypose_graph_node_indices)
      {
        geometry_msgs::Point node_position = keypose_graph->GetNodePosition(keypose_graph_node_ind);
        if (cell_has_keypose_node)
        {
          if (keypose_graph->IsNodeKeypose(keypose_graph_node_ind) &&
              viewpoint_manager->InLocalPlanningHorizon(misc_utils_ns::GeoMsgPnt2Eigen(node_position)))
          {
            double dist_to_cell_center = misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(
                node_position, cell_center_position);
            if (dist_to_cell_center < min_dist)
            {
              min_dist = dist_to_cell_center;
              min_dist_keypose_graph_node_ind = keypose_graph_node_ind;
              connection_point_set = true;
            }
          }
        }
      }
      if (connection_point_set)
      {
        geometry_msgs::Point min_dist_node_position = keypose_graph->GetNodePosition(min_dist_keypose_graph_node_ind);
        subspaces_->GetCell(cell_ind).SetRoadmapConnectionPoint(
            Eigen::Vector3d(min_dist_node_position.x, min_dist_node_position.y, min_dist_node_position.z));
        subspaces_->GetCell(cell_ind).SetRoadmapConnectionPointSet(true);
      }
    }
    if (!connection_point_set)
    {
      std::vector<int> candidate_viewpoint_indices = subspaces_->GetCell(cell_ind).GetViewPointIndices();
      if (!candidate_viewpoint_indices.empty())
      {
        double min_dist = DBL_MAX;
        int min_dist_viewpoint_ind = candidate_viewpoint_indices.front();
        for (const auto& viewpoint_ind : candidate_viewpoint_indices)
        {
          geometry_msgs::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
          double dist_to_cell_center = misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(
              viewpoint_position, cell_center_position);
          if (dist_to_cell_center < min_dist)
          {
            min_dist = dist_to_cell_center;
            min_dist_viewpoint_ind = viewpoint_ind;
          }
        }
        geometry_msgs::Point min_dist_viewpoint_position =
            viewpoint_manager->GetViewPointPosition(min_dist_viewpoint_ind);
        subspaces_->GetCell(cell_ind).SetRoadmapConnectionPoint(Eigen::Vector3d(
            min_dist_viewpoint_position.x, min_dist_viewpoint_position.y, min_dist_viewpoint_position.z));
        subspaces_->GetCell(cell_ind).SetRoadmapConnectionPointSet(true);
      }
    }
  }
}

void GridWorld::UpdateKeyposeGraph(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                   const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  for (const auto& from_cell_ind : neighbor_cell_indices_)
  {
    int from_cell_viewpoint_num = subspaces_->GetCell(from_cell_ind).GetViewPointIndices().size();
    std::vector<int> from_cell_keypose_graph_node_indices = subspaces_->GetCell(from_cell_ind).GetGraphNodeIndices();
    int from_cell_keypose_graph_node_num = from_cell_keypose_graph_node_indices.size();
    if (from_cell_viewpoint_num == 0 || from_cell_keypose_graph_node_num == 0)
    {
      continue;
    }

    Eigen::Vector3d from_cell_roadmap_connection_position =
        subspaces_->GetCell(from_cell_ind).GetRoadmapConnectionPoint();
    if (!viewpoint_manager->InLocalPlanningHorizon(from_cell_roadmap_connection_position))
    {
      continue;
    }

    Eigen::Vector3i from_cell_sub = subspaces_->Ind2Sub(from_cell_ind);
    std::vector<int> direct_neighbor_cell_indices;
    GetDirectNeighborCellIndices(from_cell_sub, direct_neighbor_cell_indices);
    for (const auto& to_cell_ind : direct_neighbor_cell_indices)
    {
      int to_cell_viewpoint_num = subspaces_->GetCell(to_cell_ind).GetViewPointIndices().size();
      if (to_cell_viewpoint_num == 0)
      {
        continue;
      }
      Eigen::Vector3d to_cell_roadmap_connection_position =
          subspaces_->GetCell(to_cell_ind).GetRoadmapConnectionPoint();
      if (!viewpoint_manager->InLocalPlanningHorizon(to_cell_roadmap_connection_position))
      {
        continue;
      }
      // Connect the two
      bool connected_in_keypose_graph = HasDirectKeyposeGraphConnection(
          keypose_graph, from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);

      if (!connected_in_keypose_graph)
      {
        nav_msgs::Path path_in_between = viewpoint_manager->GetViewPointShortestPath(
            from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);

        if (PathValid(path_in_between, from_cell_ind, to_cell_ind))
        {
          path_in_between = misc_utils_ns::SimplifyPath(path_in_between);
          for (auto& pose : path_in_between.poses)
          {
            pose.pose.orientation.w = -1;
          }
          keypose_graph->AddPath(path_in_between);
        }
      }
    }
  }
}

void GridWorld::UpdateRoadmap(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                              const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                              bool following_global_path)
{
  int robot_cell_ind = GetCellInd(robot_position_);
  Eigen::Vector3i robot_cell_sub = GetRobotCellSub();
  std::vector<int> robot_cell_direct_neighbor_indices;
  GetDirectNeighborCellIndices(robot_cell_sub, robot_cell_direct_neighbor_indices);

  for (const auto& from_cell_ind : neighbor_cell_indices_)
  {
    Eigen::Vector3i from_cell_sub = subspaces_->Ind2Sub(from_cell_ind);
    std::vector<int> direct_neighbor_cell_indices;
    GetDirectNeighborCellIndices(from_cell_sub, direct_neighbor_cell_indices);
    for (const auto& to_cell_ind : direct_neighbor_cell_indices)
    {
      bool to_cell_viewpoint_empty = subspaces_->GetCell(to_cell_ind).GetViewPointIndices().empty();
      if (roadmap_.IsConnected(from_cell_ind, to_cell_ind))
      {
        if (from_cell_ind == robot_cell_ind)
        {
          if (following_global_path && !wait_ && to_cell_viewpoint_empty && robot_stuck_in_current_cell_)
          {
            roadmap_.CutOffEdge(from_cell_ind, to_cell_ind);
          }
          else if (roadmap_.IsCutOff(from_cell_ind, to_cell_ind) && !to_cell_viewpoint_empty)
          {
            roadmap_.ResetEdgeDistance(from_cell_ind, to_cell_ind);
          }
        }
        // else if (following_global_path && !wait_ && robot_stuck_in_current_cell_ && to_cell_viewpoint_empty &&
        //          to_cell_ind != robot_cell_ind &&
        //          misc_utils_ns::ElementExistsInVector<int>(robot_cell_direct_neighbor_indices, from_cell_ind))
        // {
        //   if (!HasFrontierViewpoints(from_cell_ind, viewpoint_manager))
        //   {
        //     roadmap_.CutOffEdge(from_cell_ind, to_cell_ind);
        //   }
        // }
      }
      else
      {
        // if there is a direct keypose graph path in between, connect them on the roadmap
        Eigen::Vector3d from_cell_roadmap_connection_position =
            subspaces_->GetCell(from_cell_ind).GetRoadmapConnectionPoint();
        Eigen::Vector3d to_cell_roadmap_connection_position =
            subspaces_->GetCell(to_cell_ind).GetRoadmapConnectionPoint();
        bool connected_in_keypose_graph = HasDirectKeyposeGraphConnection(
            keypose_graph, from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);
        if (connected_in_keypose_graph)
        {
          geometry_msgs::Point from_cell_position = subspaces_->GetCell(from_cell_ind).GetPosition();
          geometry_msgs::Point to_cell_position = subspaces_->GetCell(to_cell_ind).GetPosition();
          roadmap_.AddNode(RoadmapNodeType(from_cell_ind, from_cell_position));
          roadmap_.AddNode(RoadmapNodeType(to_cell_ind, to_cell_position));
          roadmap_.AddEdge(from_cell_ind, to_cell_ind);
        }
      }
    }
  }
}

void GridWorld::UpdateKeyposeGraphAndRoadmap(
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph, bool following_global_path)
{
  SetRoadmapConnectionPointForCells(viewpoint_manager, keypose_graph);
  UpdateKeyposeGraph(viewpoint_manager, keypose_graph);
  UpdateRoadmap(viewpoint_manager, keypose_graph, following_global_path);
}

bool GridWorld::PathValid(const nav_msgs::Path& path, int from_cell_ind, int to_cell_ind)
{
  if (path.poses.size() >= 2)
  {
    for (const auto& pose : path.poses)
    {
      int cell_ind = GetCellInd(pose.pose.position);
      if (cell_ind != from_cell_ind && cell_ind != to_cell_ind)
      {
        return false;
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}

bool GridWorld::HasDirectKeyposeGraphConnection(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                                const Eigen::Vector3d& start_position,
                                                const Eigen::Vector3d& goal_position)
{
  if (!keypose_graph->HasNode(start_position) || !keypose_graph->HasNode(goal_position))
  {
    return false;
  }

  // Search a path connecting start_position and goal_position with a max path length constraint
  geometry_msgs::Point geo_start_position;
  geo_start_position.x = start_position.x();
  geo_start_position.y = start_position.y();
  geo_start_position.z = start_position.z();

  geometry_msgs::Point geo_goal_position;
  geo_goal_position.x = goal_position.x();
  geo_goal_position.y = goal_position.y();
  geo_goal_position.z = goal_position.z();

  double max_path_length = kCellSize * 2;
  nav_msgs::Path path;
  bool found_path =
      keypose_graph->GetShortestPathWithMaxLength(geo_start_position, geo_goal_position, max_path_length, false, path);
  return found_path;
}

void GridWorld::GetExplorationInfo(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                   tare_msgs::ExplorationInfo& exploration_info)
{
  exploration_info.cells.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    bool skip = false;
    if (subspaces_->GetCell(i).IsSynced() && subspaces_->GetCell(i).GetSyncCount() >= Cell::kSyncDelay)
    {
      skip = true;
    }
    if (misc_utils_ns::ElementExistsInVector<int>(global_exploring_cell_indices_, i))
    {
      skip = false;
    }

    if (skip)
    {
      continue;
    }

    CellStatus cell_status = subspaces_->GetCell(i).GetStatus();

    geometry_msgs::Point connected_point;
    if (cell_status == CellStatus::COVERED || cell_status == CellStatus::COVERED_BY_OTHERS || IsCellCoveredToOthers(i))
    {
      tare_msgs::Cell cell = ToCellMsg(i, CellStatus::COVERED);
      exploration_info.cells.push_back(cell);
      if (IsCellSynced(i))
      {
        subspaces_->GetCell(i).AddSyncCount();
      }
    }
    else if ((cell_status == CellStatus::EXPLORING && IsConnectedOnKeyposeGraph(i, keypose_graph, connected_point)) ||
             (cell_status == CellStatus::EXPLORING_BY_OTHERS && IsConnectedOnRoadmap(i)))
    {
      tare_msgs::Cell cell = ToCellMsg(i, CellStatus::EXPLORING);
      exploration_info.cells.push_back(cell);
      if (IsCellSynced(i))
      {
        subspaces_->GetCell(i).AddSyncCount();
      }
    }
  }

  // Get global VRP plan
  exploration_info.global_cell_ids.clear();

  int32_t sampled_robots = SampleRobotVectorToInt(is_sampled_last_iteration_);
  exploration_info.global_cell_ids.push_back(sampled_robots);

  if (!no_comms_global_plan_.empty())
  {
    for (const auto& row : no_comms_global_plan_)
    {
      for (const auto& cell_id : row)
      {
        exploration_info.global_cell_ids.push_back(cell_id);
      }
      exploration_info.global_cell_ids.push_back(-1);
    }
  }

  if (kRendezvous)
  {
    exploration_info.global_cell_ids.push_back(-2);
    exploration_info.global_cell_ids.push_back(rendezvous_manager_.GetNextRendezvousCellID());
    exploration_info.global_cell_ids.push_back(rendezvous_manager_.GetNextRendezvousTimeInterval());
    exploration_info.global_cell_ids.push_back(-1);
    exploration_info.global_cell_ids.push_back(-2);
  }
  else
  {
    if (!relay_comms_global_plan_.empty())
    {
      exploration_info.global_cell_ids.push_back(-2);
      for (const auto& row : relay_comms_global_plan_)
      {
        for (const auto& cell_id : row)
        {
          exploration_info.global_cell_ids.push_back(cell_id);
        }
        exploration_info.global_cell_ids.push_back(-1);
      }
    }

    if (!assume_comms_global_plan_.empty())
    {
      exploration_info.global_cell_ids.push_back(-2);
      for (const auto& row : assume_comms_global_plan_)
      {
        for (const auto& cell_id : row)
        {
          exploration_info.global_cell_ids.push_back(cell_id);
        }
        exploration_info.global_cell_ids.push_back(-1);
      }
    }
  }

  // Push back the local_exploring_cell_indices_
  exploration_info.local_cell_ids.clear();
  // std::cout << "local exploring cell num: " << local_exploring_cell_indices_.size() << std::endl;
  for (const auto& cell_id : local_exploring_cell_indices_)
  {
    exploration_info.local_cell_ids.push_back(cell_id);
  }

  // Get all the roadmap edges
  exploration_info.roadmap_edges.clear();
  std::vector<std::pair<int, int>> edges;
  roadmap_.GetEdges(edges);
  for (int i = 0; i < edges.size(); i++)
  {
    tare_msgs::Edge edge;
    int from_cell_id = edges[i].first;
    int to_cell_id = edges[i].second;
    edge.from_cell_id = from_cell_id;
    edge.to_cell_id = to_cell_id;
    edge.weight = EncodeRoadmapEdgeWeight(GetCellMobilityType(from_cell_id), GetCellMobilityType(to_cell_id));
    exploration_info.roadmap_edges.push_back(edge);
  }
}

void GridWorld::GetOrderedExploringCellIndices(RouteInCellIndices& ordered_cell_indices)
{
  ordered_cell_indices.clear();
  ordered_cell_indices = no_comms_global_plan_;
  // ordered_cell_indices = relay_comms_global_plan_;
}

std::string GridWorld::GetCellIDLabel(int cell_id)
{
  std::string cell_label = std::to_string(cell_id);
  cell_label = cell_label.substr(cell_label.size() - 3);
  return cell_label;
}

void GridWorld::SyncCellWithIDs(int cell_id, int ids, CellStatus cell_status)
{
  for (int robot_id = 0; robot_id < kRobotNum; robot_id++)
  {
    if (ids & (1 << robot_id))
    {
      subspaces_->GetCell(cell_id).AddSyncedCount(cell_status, robot_id);
    }
  }
  SyncCellWithIDs(cell_id, ids);

  if (IsCellSynced(cell_id))
  {
    for (int i = 0; i < kRobotNum; i++)
    {
      if (i != kRobotID)
      {
        int synced_count = subspaces_->GetCell(cell_id).GetSyncedCount(i);
        if (synced_count > Cell::kSyncDelay)
        {
          subspaces_->GetCell(cell_id).ReduceSyncCount();
          subspaces_->GetCell(cell_id).ResetSyncedCount(i);
        }
      }
    }
  }
}

void GridWorld::SyncCellWithIDs(int cell_id, int ids)
{
  MY_ASSERT(subspaces_->InRange(cell_id));
  subspaces_->GetCell(cell_id).SyncIDs(ids);
}

void GridWorld::LoadRoadmapFromFile()
{
  int robot_id = 0;
  std::string common_path = ros::package::getPath("tare_planner") + "/log/";
  std::string nodes_filename = common_path + "robot" + std::to_string(robot_id) + "_nodes.txt";
  std::string connection_matrix_filename = common_path + "robot" + std::to_string(robot_id) + "_connection_matrix.txt";
  std::string distance_matrix_filename = common_path + "robot" + std::to_string(robot_id) + "_distance_matrix.txt";

  roadmap_.ReadFromFile(nodes_filename, connection_matrix_filename, distance_matrix_filename);
}

bool GridWorld::CheckDistanceMatrixSize(const std::vector<std::vector<int>>& distance_matrix, int dim)
{
  if (distance_matrix.size() != dim)
  {
    return false;
  }
  for (int i = 0; i < distance_matrix.size(); i++)
  {
    if (distance_matrix[i].size() != dim)
    {
      return false;
    }
  }
  return true;
}

bool GridWorld::IsConnectedOnKeyposeGraph(int cell_ind,
                                          const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                          geometry_msgs::Point& connected_point)
{
  if (keypose_graph == nullptr)
  {
    return false;
  }

  connected_point = misc_utils_ns::Eigen2GeoMsgPnt(subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint());
  if (keypose_graph->IsPositionReachable(connected_point))
  {
    return true;
  }
  else
  {
    // Go through all the keypose graph nodes in the cell, find the nearest one to the previous connected position
    double min_dist = DBL_MAX;
    double min_dist_node_ind = -1;
    for (const auto& node_ind : subspaces_->GetCell(cell_ind).GetGraphNodeIndices())
    {
      geometry_msgs::Point node_position = keypose_graph->GetNodePosition(node_ind);
      double dist =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(node_position, connected_point);
      if (dist < min_dist)
      {
        min_dist = dist;
        min_dist_node_ind = node_ind;
      }
    }
    if (min_dist_node_ind >= 0 && min_dist_node_ind < keypose_graph->GetNodeCount())
    {
      connected_point = keypose_graph->GetNodePosition(min_dist_node_ind);
      return true;
    }
    else
    {
      return false;
    }
  }
}

bool GridWorld::IsConnectedOnRoadmap(int cell_ind)
{
  if (!roadmap_.HasNode(cell_ind))
  {
    return false;
  }
  std::vector<double> neighbor_distances;
  roadmap_.GetNeighborDistances(cell_ind, neighbor_distances);
  bool has_traversable_edge = false;
  double edge_cutoff_distance_threshold = roadmap_.GetEdgeCutOffDistanceThreshold();
  for (const auto& neighbor_dist : neighbor_distances)
  {
    if (neighbor_dist < edge_cutoff_distance_threshold)
    {
      has_traversable_edge = true;
      break;
    }
  }
  return has_traversable_edge;
}

int GridWorld::CellToCellDistance(int from_cell_ind, int to_cell_ind,
                                  const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  // Use roadmap

  // geometry_msgs::Point from_cell_connected_point;
  // geometry_msgs::Point to_cell_connected_point;
  // bool from_cell_connected_on_keypose_graph =
  //     IsConnectedOnKeyposeGraph(from_cell_ind, keypose_graph, from_cell_connected_point);
  // bool to_cell_connected_on_keypose_graph =
  //     IsConnectedOnKeyposeGraph(to_cell_ind, keypose_graph, to_cell_connected_point);

  double path_length = 0;
  // if (from_cell_connected_on_keypose_graph && to_cell_connected_on_keypose_graph)
  // {
  //   nav_msgs::Path path;
  //   path_length = keypose_graph->GetShortestPath(from_cell_connected_point, to_cell_connected_point, false, path);
  // }
  // else
  {
    bool from_cell_connected_on_roadmap = IsConnectedOnRoadmap(from_cell_ind);
    bool to_cell_connected_on_roadmap = IsConnectedOnRoadmap(to_cell_ind);
    // if (!from_cell_connected_on_roadmap)
    // {
    //   ROS_ERROR_STREAM("From cell: " << from_cell_ind << " is not connected on roadmap");
    // }
    // if (!to_cell_connected_on_roadmap)
    // {
    //   ROS_ERROR_STREAM("To cell: " << to_cell_ind << " is not connected on roadmap");
    // }
    if (from_cell_connected_on_roadmap && to_cell_connected_on_roadmap)
    {
      nav_msgs::Path path;
      std::vector<int> node_ids;
      path_length = roadmap_.GetShortestPath(from_cell_ind, to_cell_ind, false, path, node_ids);
    }
    else
    {
      // Get straight line distance
      geometry_msgs::Point from_cell_position = GetCellPosition(from_cell_ind);
      geometry_msgs::Point to_cell_position = GetCellPosition(to_cell_ind);
      path_length =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(from_cell_position, to_cell_position);
    }
  }

  return static_cast<int>(path_length);
}

tare::VRPCost GridWorld::SolveVRP(const DistanceMatrix& distance_matrices, const RouteInDMIndices& initial_routes,
                                  RouteInDMIndices& vrp_solution)
{
  int robot_num = distance_matrices.size();
  tsp_solver_ns::DataModel data_model;
  data_model.num_vehicles = robot_num;
  data_model.distance_matrices = distance_matrices;
  data_model.initial_routes = initial_routes;
  for (int i = 0; i < robot_num; i++)
  {
    data_model.depots.push_back(operations_research::RoutingIndexManager::NodeIndex{ i });
  }
  tsp_solver_ns::TSPSolver solver(data_model);
  if (kUseTimeBudget)
  {
    int remaining_exploration_time = time_budget_manager_.GetRemainingExplorationTime();
    int64 max_route_length = static_cast<int64>(remaining_exploration_time * kRobotSpeed);
    // ROS_INFO_STREAM("Remaining exploration time: " << remaining_exploration_time);
    // ROS_INFO_STREAM("max route length: " << max_route_length);
    solver.SolveVRP(max_route_length);
  }
  else
  {
    solver.SolveVRP();
  }
  solver.GetVRPSolutionNodeIndex(vrp_solution);

  std::vector<bool> is_sampled(data_model.num_vehicles, false);
  std::vector<int> sampled_node_indices(data_model.num_vehicles, -1);
  int longest_route_distance =
      GetLongestRouteDistance(distance_matrices, vrp_solution, is_sampled, sampled_node_indices);

  return tare::VRPCost(solver.GetDroppedNodesNumber(), longest_route_distance);
}

std::pair<int, int> GridWorld::SolveVRPWithTimeWindowConstraint(int robot_num, const DistanceMatrix& distance_matrices,
                                                                const std::vector<std::pair<int, int>>& time_windows,
                                                                const std::vector<bool>& is_sampled,
                                                                const std::vector<int>& sampled_node_indices,
                                                                const RouteInDMIndices& initial_routes,
                                                                RouteInCellIndices& ordered_exploring_cell_ids)
{
  tsp_solver_ns::DataModel data_model;
  data_model.num_vehicles = robot_num;
  data_model.distance_matrices = distance_matrices;
  data_model.initial_routes = initial_routes;
  data_model.time_windows = time_windows;
  for (int i = 0; i < robot_num; i++)
  {
    data_model.depots.push_back(operations_research::RoutingIndexManager::NodeIndex{ i });
  }
  tsp_solver_ns::TSPSolver solver(data_model);
  solver.SolveVRP();
  solver.GetVRPSolutionNodeIndex(ordered_exploring_cell_ids);

  int longest_route_distance =
      GetLongestRouteDistance(distance_matrices, ordered_exploring_cell_ids, is_sampled, sampled_node_indices);
  return std::make_pair(solver.GetDroppedNodesNumber(), longest_route_distance);
}

exploration_path_ns::ExplorationPath
GridWorld::GetReturnHomePath(const geometry_msgs::Point& global_path_robot_position,
                             const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  exploration_path_ns::ExplorationPath return_home_path;

  geometry_msgs::Point home_position;
  home_position.x = 0;
  home_position.y = 0;
  home_position.z = 0;
  nav_msgs::Path nav_msgs_path;

  if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeCount() == 0)
  {
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.pose.position = robot_position_;

    geometry_msgs::PoseStamped home_pose;
    home_pose.pose.position = home_position;
    nav_msgs_path.poses.push_back(robot_pose);
    nav_msgs_path.poses.push_back(home_pose);
  }
  else
  {
    home_position = keypose_graph->GetFirstKeyposePosition();
    keypose_graph->GetShortestPath(global_path_robot_position, home_position, true, nav_msgs_path, false);
    if (nav_msgs_path.poses.size() >= 2)
    {
      return_home_path.FromPath(nav_msgs_path);
      return_home_path.nodes_.front().type_ = exploration_path_ns::NodeType::ROBOT;

      for (int i = 1; i < return_home_path.nodes_.size() - 1; i++)
      {
        return_home_path.nodes_[i].type_ = exploration_path_ns::NodeType::GLOBAL_VIA_POINT;
      }
      return_home_path.nodes_.back().type_ = exploration_path_ns::NodeType::HOME;
      // Make it a loop
      for (int i = return_home_path.nodes_.size() - 2; i >= 0; i--)
      {
        return_home_path.Append(return_home_path.nodes_[i]);
      }
    }
    else
    {
      // ROS_ERROR("Cannot find path home");
      // TODO: find a path
    }
  }
  return return_home_path;
}

exploration_path_ns::ExplorationPath
GridWorld::GetGlobalExplorationPath(const std::vector<int>& ordered_exploring_cell_ids,
                                    const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  // There should be more than one cell
  MY_ASSERT(ordered_exploring_cell_ids.size() > 1);
  // The first cell should be the one where the robot is in
  MY_ASSERT(GetCellInd(robot_position_) == ordered_exploring_cell_ids.front());

  // Reset roadmap traversability
  UpdateRoadmapNodeTraversability(self_mobility_type_);

  exploration_path_ns::ExplorationPath global_path;

  for (int i = 0; i < ordered_exploring_cell_ids.size() - 1; i++)
  {
    int from_cell_id = ordered_exploring_cell_ids[i];
    int to_cell_id = ordered_exploring_cell_ids[i + 1];
    AddPathSegment(from_cell_id, to_cell_id, keypose_graph, global_path);
  }
  global_path.nodes_.front().type_ = exploration_path_ns::NodeType::ROBOT;
  global_path.nodes_.back().type_ = exploration_path_ns::NodeType::ROBOT;

  return global_path;
}

geometry_msgs::Point
GridWorld::GetGlobalPathRobotPosition(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                      const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  geometry_msgs::Point global_path_robot_position = robot_position_;

  Eigen::Vector3d eigen_robot_position(robot_position_.x, robot_position_.y, robot_position_.z);
  // Get nearest connected node
  double min_dist_to_robot = DBL_MAX;
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  keypose_graph->GetClosestConnectedNodeIndAndDistance(robot_position_, closest_node_ind, closest_node_dist);
  if (closest_node_dist < kCellSize / 2 && closest_node_ind >= 0 && closest_node_ind < keypose_graph->GetNodeCount())
  {
    global_path_robot_position = keypose_graph->GetNodePosition(closest_node_ind);
  }
  else if (cur_keypose_graph_node_ind_ >= 0 && cur_keypose_graph_node_ind_ < keypose_graph->GetNodeCount())
  {
    global_path_robot_position = keypose_graph->GetNodePosition(cur_keypose_graph_node_ind_);
  }
  else
  {
    for (int i = 0; i < neighbor_cell_indices_.size(); i++)
    {
      int cell_ind = neighbor_cell_indices_[i];
      if (subspaces_->GetCell(cell_ind).IsRoadmapConnectionPointSet())
      {
        Eigen::Vector3d roadmap_connection_point = subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint();
        if (viewpoint_manager->InLocalPlanningHorizon(roadmap_connection_point))
        {
          double dist_to_robot = (roadmap_connection_point - eigen_robot_position).norm();
          if (dist_to_robot < min_dist_to_robot)
          {
            min_dist_to_robot = dist_to_robot;
            global_path_robot_position.x = roadmap_connection_point.x();
            global_path_robot_position.y = roadmap_connection_point.y();
            global_path_robot_position.z = roadmap_connection_point.z();
          }
        }
      }
    }
  }
  return global_path_robot_position;
}

void GridWorld::PrintDistanceMatrix(int robot_num, const std::vector<int>& cell_ids,
                                    const DistanceMatrix& distance_matrices)
{
  std::cout << "-----Distance Matrix Begins-----" << std::endl;
  for (int robot_id = 0; robot_id < robot_num; robot_id++)
  {
    std::cout << "Robot " << robot_id << " distance matrix: " << std::endl;
    // Print the headers
    std::cout << "\t";
    for (int i = 0; i < robot_num; i++)
    {
      std::cout << "R" << std::to_string(i) << "\t";
    }
    for (int i = 0; i < cell_ids.size(); i++)
    {
      std::cout << GetCellIDLabel(cell_ids[i]) << "\t";
    }
    std::cout << std::endl;
    for (int i = 0; i < distance_matrices[robot_id].size(); i++)
    {
      if (i < robot_num)
      {
        std::cout << "R" << std::to_string(i) << "\t";
      }
      else
      {
        std::cout << GetCellIDLabel(cell_ids[i - robot_num]) << "\t";
      }
      for (int j = 0; j < distance_matrices[robot_id][i].size(); j++)
      {
        std::cout << distance_matrices[robot_id][i][j] << "\t";
      }
      std::cout << std::endl;
    }
    // std::cout << ""
  }

  std::cout << "-----Distance Matrix Ends-----" << std::endl;
}

void GridWorld::AddPathSegment(int from_cell_id, int to_cell_id,
                               const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                               exploration_path_ns::ExplorationPath& global_path)
{
  bool from_cell_connected_on_roadmap = IsConnectedOnRoadmap(from_cell_id);
  bool to_cell_connected_on_roadmap = IsConnectedOnRoadmap(to_cell_id);

  // Check if the cell is connected on the keypose_graph, if so, use keypose graph to find paths between cells
  // Otherwise, use the roadmap
  geometry_msgs::Point from_cell_connected_point;
  geometry_msgs::Point to_cell_connected_point;

  bool from_cell_connected_on_keypose_graph = false;
  bool to_cell_connected_on_keypose_graph = false;

  int robot_cell_id = GetCellInd(robot_position_);
  if (from_cell_id == robot_cell_id)
  {
    from_cell_connected_on_keypose_graph = true;
    if (cur_keypose_graph_node_ind_ >= 0 && cur_keypose_graph_node_ind_ < keypose_graph->GetNodeCount())
    {
      from_cell_connected_point = keypose_graph->GetNodePosition(cur_keypose_graph_node_ind_);
    }
  }
  else
  {
    from_cell_connected_on_keypose_graph =
        IsConnectedOnKeyposeGraph(from_cell_id, keypose_graph, from_cell_connected_point);
  }

  if (to_cell_id == robot_cell_id)
  {
    to_cell_connected_on_keypose_graph = true;
    if (cur_keypose_graph_node_ind_ >= 0 && cur_keypose_graph_node_ind_ < keypose_graph->GetNodeCount())
    {
      to_cell_connected_point = keypose_graph->GetNodePosition(cur_keypose_graph_node_ind_);
    }
  }
  else
  {
    to_cell_connected_on_keypose_graph = IsConnectedOnKeyposeGraph(to_cell_id, keypose_graph, to_cell_connected_point);
  }

  if (from_cell_connected_on_keypose_graph && to_cell_connected_on_keypose_graph)
  {
    // Use keypose graph
    nav_msgs::Path keypose_path;
    keypose_graph->GetShortestPath(from_cell_connected_point, to_cell_connected_point, true, keypose_path, true);

    for (int i = 0; i < keypose_path.poses.size(); i++)
    {
      geometry_msgs::Point cur_position = keypose_path.poses[i].pose.position;
      exploration_path_ns::Node node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
      if (i == 0 || i == keypose_path.poses.size() - 1)
      {
        node.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;
      }
      else
      {
        node.type_ = exploration_path_ns::NodeType::GLOBAL_VIA_POINT;
      }
      node.global_subspace_index_ = GetCellInd(keypose_path.poses[i].pose.position);
      global_path.Append(node);
    }
  }
  else if (from_cell_connected_on_roadmap && to_cell_connected_on_roadmap)
  {
    nav_msgs::Path roadmap_path;
    std::vector<int> on_path_cell_indices;

    roadmap_.GetShortestPath(from_cell_id, to_cell_id, true, roadmap_path, on_path_cell_indices, false);
    if (on_path_cell_indices.size() >= 2)
    {
      for (int i = 0; i < on_path_cell_indices.size() - 1; i++)
      {
        int curr_cell_id = on_path_cell_indices[i];
        int next_cell_id = on_path_cell_indices[i + 1];
        geometry_msgs::Point curr_cell_connected_point;
        geometry_msgs::Point next_cell_connected_point;
        bool curr_cell_connected = IsConnectedOnKeyposeGraph(curr_cell_id, keypose_graph, curr_cell_connected_point);
        bool next_cell_connected = IsConnectedOnKeyposeGraph(next_cell_id, keypose_graph, next_cell_connected_point);
        // TODO: use get shortest path with max length
        bool got_keypose_path = false;
        if (curr_cell_connected && next_cell_connected)
        {
          nav_msgs::Path keypose_path;
          got_keypose_path = keypose_graph->GetShortestPathWithMaxLength(
              curr_cell_connected_point, next_cell_connected_point, kCellSize * 3, true, keypose_path);
          if (got_keypose_path)
          {
            for (int j = 0; j < keypose_path.poses.size(); j++)
            {
              geometry_msgs::Point cur_position = keypose_path.poses[j].pose.position;
              exploration_path_ns::Node node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
              node.type_ = exploration_path_ns::NodeType::GLOBAL_VIA_POINT;
              if ((i == 0 && j == 0) ||
                  ((i + 1) == (on_path_cell_indices.size() - 1) && (j == keypose_path.poses.size() - 1)))
              {
                node.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;
              }
              node.global_subspace_index_ = GetCellInd(keypose_path.poses[j].pose.position);
              global_path.Append(node);
            }
          }
        }

        if (!got_keypose_path)
        {
          geometry_msgs::Point cur_position = GetCellPosition(curr_cell_id);
          exploration_path_ns::Node node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
          node.type_ = exploration_path_ns::NodeType::GLOBAL_ROADMAP;
          node.global_subspace_index_ = curr_cell_id;
          global_path.Append(node);
          if (i == on_path_cell_indices.size() - 2)
          {
            int curr_cell_id = on_path_cell_indices.back();
            geometry_msgs::Point cur_position = GetCellPosition(curr_cell_id);
            exploration_path_ns::Node node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
            node.type_ = exploration_path_ns::NodeType::GLOBAL_ROADMAP;
            node.global_subspace_index_ = curr_cell_id;
            global_path.Append(node);
          }
        }
      }
    }
    else if (on_path_cell_indices.size() == 1)
    {
      int curr_cell_id = on_path_cell_indices.front();
      geometry_msgs::Point cur_position = GetCellPosition(curr_cell_id);
      exploration_path_ns::Node node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
      node.type_ = exploration_path_ns::NodeType::GLOBAL_ROADMAP;
      node.global_subspace_index_ = curr_cell_id;
      global_path.Append(node);
    }
    else
    {
      // Cannot find a path
      ROS_ERROR_STREAM("Cannot find a path between " << from_cell_id << " and " << to_cell_id
                                                     << " using straight line as a path");
      geometry_msgs::Point cur_position = GetCellPosition(from_cell_id);
      exploration_path_ns::Node from_cell_node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
      from_cell_node.type_ = exploration_path_ns::NodeType::GLOBAL_ROADMAP;
      from_cell_node.global_subspace_index_ = from_cell_id;
      global_path.Append(from_cell_node);

      cur_position = GetCellPosition(to_cell_id);
      exploration_path_ns::Node to_cell_node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
      to_cell_node.type_ = exploration_path_ns::NodeType::GLOBAL_ROADMAP;
      to_cell_node.global_subspace_index_ = to_cell_id;
      global_path.Append(to_cell_node);
    }
  }
  else
  {
    geometry_msgs::Point cur_position = GetCellPosition(from_cell_id);
    exploration_path_ns::Node from_cell_node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
    from_cell_node.type_ = exploration_path_ns::NodeType::GLOBAL_ROADMAP;
    from_cell_node.global_subspace_index_ = from_cell_id;
    global_path.Append(from_cell_node);

    cur_position = GetCellPosition(to_cell_id);
    exploration_path_ns::Node to_cell_node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
    to_cell_node.type_ = exploration_path_ns::NodeType::GLOBAL_ROADMAP;
    to_cell_node.global_subspace_index_ = to_cell_id;
    global_path.Append(to_cell_node);
  }
}

void GridWorld::UpdateRoadmapNodeTraversability(tare::MobilityType robot_mobility_type)
{
  int node_num = roadmap_.GetNodeNumber();
  for (int i = 0; i < node_num; i++)
  {
    int cell_ind = roadmap_.GetNodeID(i);
    roadmap_.GetNodeByIndex(i).traversable_ = tare::Traversable(robot_mobility_type, GetCellMobilityType(cell_ind));
  }
}

double GridWorld::EncodeRoadmapEdgeWeight(tare::MobilityType from_mobility_type, tare::MobilityType to_mobility_type)
{
  return static_cast<int>(from_mobility_type) * 10 + static_cast<int>(to_mobility_type);
}

void GridWorld::DecodeRoadmapEdgeWeight(double weight, tare::MobilityType& from_mobility_type,
                                        tare::MobilityType& to_mobility_type)
{
  int type = static_cast<int>(weight);
  to_mobility_type = static_cast<tare::MobilityType>(type % 10);
  type /= 10;
  from_mobility_type = static_cast<tare::MobilityType>(type);
}

void GridWorld::SaveGlobalPlanningStatusToFile(const std::vector<tare::Robot>& robots)
{
  // Save roadmap
  std::string common_path = ros::package::getPath("tare_planner") + "/log/";
  std::string connection_matrix_filename = common_path + "robot" + std::to_string(kRobotID) + "_connection_matrix.txt";
  std::string distance_matrix_filename = common_path + "robot" + std::to_string(kRobotID) + "_distance_matrix.txt";
  std::string node_filename = common_path + "robot" + std::to_string(kRobotID) + "_nodes.txt";
  roadmap_.SaveToFile(node_filename, connection_matrix_filename, distance_matrix_filename);

  // Save exploring cell ids and robot cell ids
  std::string exploring_cell_ids_filename =
      common_path + "robot" + std::to_string(kRobotID) + "_exploring_cell_ids.txt";
  misc_utils_ns::SaveVectorToFile<int>(global_exploring_cell_indices_, exploring_cell_ids_filename);

  std::string ordered_exploring_cell_ids_filename =
      common_path + "robot" + std::to_string(kRobotID) + "_ordered_exploring_cell_ids.txt";
  misc_utils_ns::Save2DVectorToFile<int>(no_comms_global_plan_, ordered_exploring_cell_ids_filename);

  std::string robot_cell_ids_filename = common_path + "robot" + std::to_string(kRobotID) + "_robot_cell_ids.txt";
  std::vector<int> robot_cell_ids;
  for (const auto& robot : robots)
  {
    robot_cell_ids.push_back(GetCellInd(robot.position_));
  }
  misc_utils_ns::SaveVectorToFile<int>(robot_cell_ids, robot_cell_ids_filename);
}

void GridWorld::UpdateNeighboringCellPriority(const std::vector<tare::Robot>& robots)
{
  for (int i = 0; i < neighbor_cell_indices_.size(); i++)
  {
    int cell_id = neighbor_cell_indices_[i];
    SetCellHasExploringPriority(cell_id, true);
  }

  for (int i = 0; i < kRobotID; i++)
  {
    if (!robots[i].in_comms_)
    {
      continue;
    }
    std::vector<int> peer_robot_neighboring_cell_ids = robots[i].vrp_plan_.local_exploring_cell_ids_;

    for (int j = 0; j < peer_robot_neighboring_cell_ids.size(); j++)
    {
      int cell_id = peer_robot_neighboring_cell_ids[j];
      if (misc_utils_ns::ElementExistsInVector<int>(neighbor_cell_indices_, cell_id))
      {
        SetCellHasExploringPriority(cell_id, false);
      }
    }
  }
}

int GridWorld::GetSelfExploringCellsCount()
{
  if (no_comms_global_plan_.empty())
  {
    return 0;
  }
  else
  {
    MY_ASSERT(no_comms_global_plan_[kRobotID].size() >= 2);
    return no_comms_global_plan_[kRobotID].size() - 2;
  }
}

void GridWorld::AddToAlmostCovered(int cell_ind)
{
  almost_covered_cell_indices_.push_back(cell_ind);
}

void GridWorld::RemoveFromAlmostCovered(int cell_ind)
{
  almost_covered_cell_indices_.erase(
      std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
      almost_covered_cell_indices_.end());
}

void GridWorld::GetInCommsRobotIDs(const std::vector<tare::Robot>& robots, std::vector<int>& in_comms_robot_ids)
{
  in_comms_robot_ids.clear();
  for (const auto& robot : robots)
  {
    if (robot.in_comms_)
    {
      in_comms_robot_ids.push_back(robot.id_);
    }
  }
}

/**
 * @brief
 *
 * @param pursuit_robot_ids
 * @param pursuit_cell_ids
 * @param arrival_probability
 * @param pursuit_distance_matrix: dimension: pursuit_robot_ids.size() + pursuit_cell_ids.size()
 * @param pursuit_paths_in_dm_indices
 */
void GridWorld::SolveTOPwTVP(const std::vector<int>& pursuit_robot_ids, const std::vector<int>& pursuit_cell_ids,
                             const std::vector<std::vector<double>>& arrival_probability,
                             const std::vector<std::vector<int>>& pursuit_distance_matrix, int time_budget,
                             std::vector<std::vector<int>>& pursuit_paths_in_dm_indices)
{
  if (pursuit_robot_ids.empty() || pursuit_cell_ids.empty() || pursuit_distance_matrix.empty() ||
      arrival_probability.empty())
  {
    pursuit_paths_in_dm_indices.clear();
    return;
  }
  int node_num = pursuit_robot_ids.size() + pursuit_cell_ids.size();
  int vehicle_num = pursuit_robot_ids.size();
  int max_time_steps = arrival_probability[0].size();

  // Augment arrival probability for the start nodes
  std::vector<std::vector<double>> augmented_arrival_probability;
  for (int i = 0; i < vehicle_num; i++)
  {
    augmented_arrival_probability.push_back(std::vector<double>(max_time_steps, 0));
  }
  for (int i = 0; i < arrival_probability.size(); i++)
  {
    augmented_arrival_probability.push_back(arrival_probability[i]);
  }

  TeamOrienteeringProblemSolver TOPwTVR_solver(node_num, pursuit_robot_ids.size(), time_budget, pursuit_distance_matrix,
                                               augmented_arrival_probability);

  TOPwTVR_solver.SetFrontierQueueSize(1);
  TOPwTVR_solver.SetUseClosedSet(true);
  TOPwTVR_solver.SetHeuristicType(HeuristicType::SIMPLE);
  TOPwTVR_solver.SetAccumulateRewardAtSameNode(true);
  TOPwTVR_solver.SetTimeBudget(time_budget);

  double greedy_total_profit = 0;
  std::vector<int> start_node_ids;
  std::vector<int> end_goal_ids;
  for (int i = 0; i < vehicle_num; i++)
  {
    start_node_ids.push_back(i);
    end_goal_ids.push_back(0);
  }

  TOPwTVR_solver.SetStartGraphNodeIDs(start_node_ids);
  TOPwTVR_solver.SetGoalGraphNodeIDs(end_goal_ids);

  misc_utils_ns::Timer timer_TOPwTVR("solve TOPwTVR");
  timer_TOPwTVR.Start();
  greedy_total_profit = TOPwTVR_solver.SolveMultiVehiclePBS(pursuit_paths_in_dm_indices);
  timer_TOPwTVR.Stop(false);
  int solve_TOPwTVR_time = timer_TOPwTVR.GetDuration();
  if (solve_TOPwTVR_time > 2000)
  {
    // Save pursuit_distance_matrix augmented_arrival_probability to files
    std::string common_path = ros::package::getPath("tare_planner") + "/data/";
    std::string distance_matrix_filename = common_path + "robot" + std::to_string(kRobotID) + "_distance_matrix.txt";
    std::string arrival_probability_filename =
        common_path + "robot" + std::to_string(kRobotID) + "_" + std::to_string(node_num) + "_" +
        std::to_string(pursuit_robot_ids.size()) + "_" + "_arrival_probability.txt";
    misc_utils_ns::Save2DVectorToFile<int>(pursuit_distance_matrix, distance_matrix_filename);
    misc_utils_ns::Save2DVectorToFile<double>(augmented_arrival_probability, arrival_probability_filename);
  }
}

void GridWorld::VisualizePursuitPaths(const std::vector<int>& pursuit_robot_ids,
                                      const std::vector<int>& pursuit_cell_ids, const std::vector<tare::Robot>& robots,
                                      const std::vector<std::vector<int>>& pursuit_distance_matrix,
                                      std::vector<std::vector<int>>& pursuit_paths)
{
  int pursuit_robot_num = pursuit_robot_ids.size();
  for (int i = 0; i < pursuit_paths.size(); i++)
  {
    nav_msgs::Path single_robot_path;
    single_robot_path.header.frame_id = "map";
    single_robot_path.header.stamp = ros::Time::now();

    int accumulated_t = -1;
    for (int j = 0; j < pursuit_paths[i].size(); j++)
    {
      int delta_t = 1;
      int curr_node_id = pursuit_paths[i][j];
      if (j > 0)
      {
        int prev_node_id = pursuit_paths[i][j - 1];
        if (curr_node_id != prev_node_id)
        {
          delta_t = pursuit_distance_matrix[prev_node_id][curr_node_id];
        }
      }
      accumulated_t += delta_t;

      geometry_msgs::PoseStamped node_pose;
      if (curr_node_id < pursuit_robot_num)
      {
        int robot_id = pursuit_robot_ids[curr_node_id];
        node_pose.pose.position = robots[robot_id].in_comms_position_;
        node_pose.pose.position.z += accumulated_t * 1.5;
      }
      else
      {
        int cell_id = pursuit_cell_ids[curr_node_id - pursuit_robot_num];
        node_pose.pose.position = GetCellPosition(cell_id);
        node_pose.pose.position.z += accumulated_t * 1.5;
      }

      single_robot_path.poses.push_back(node_pose);
    }

    int robot_id = pursuit_robot_ids[i];
    TOPwTVR_path_publishers_[robot_id].publish(single_robot_path);
  }
}

void GridWorld::ConvertToRouteInDMIndices(const std::vector<int>& pursuit_robot_ids,
                                          const std::vector<int>& pursuit_cell_ids,
                                          const std::vector<std::vector<int>>& TOPwTVR_paths,
                                          RouteInDMIndices& relay_comms_TOPwTVP_solution)
{
  relay_comms_TOPwTVP_solution.clear();
  relay_comms_TOPwTVP_solution.resize(kRobotNum, std::vector<int>());
  for (int i = 0; i < kRobotNum; i++)
  {
    relay_comms_TOPwTVP_solution[i].push_back(i);
    relay_comms_TOPwTVP_solution[i].push_back(i);
  }

  MY_ASSERT(pursuit_robot_ids.size() == TOPwTVR_paths.size());
  for (int i = 0; i < pursuit_robot_ids.size(); i++)
  {
    int robot_id = pursuit_robot_ids[i];
    relay_comms_TOPwTVP_solution[robot_id].clear();
    relay_comms_TOPwTVP_solution[robot_id].push_back(robot_id);
    for (int j = 0; j < TOPwTVR_paths[i].size(); j++)
    {
      int node_id = TOPwTVR_paths[i][j];
      if (node_id >= pursuit_robot_ids.size())
      {
        int cell_id = pursuit_cell_ids[node_id - pursuit_robot_ids.size()];
        int cell_dm_index = cell_id_to_dm_index_[cell_id];
        if (cell_dm_index != relay_comms_TOPwTVP_solution[robot_id].back())
        {
          relay_comms_TOPwTVP_solution[robot_id].push_back(cell_dm_index);
        }
      }
    }
    relay_comms_TOPwTVP_solution[robot_id].push_back(robot_id);
  }
}

void GridWorld::ComputePursuitRoute(const DistanceMatrix& distance_matrices_with_traversability,
                                    const std::vector<tare::Robot>& robots, const std::vector<bool>& is_sampled,
                                    RouteInDMIndices& relay_comms_TOPwTVP_solution)
{
  relay_comms_TOPwTVP_solution.clear();

  // Compute the arrival probability over time at each exploring cells that are going to be visited by sampled
  // out-of-comms robots
  double time_resolution = 6.0;  // Assuming 1 m/s speed
  int max_time_steps = 100;

  std::vector<int> pursuit_cell_ids;
  std::vector<std::vector<double>> arrival_probability;
  ComputeArrivalProbabilityOutOfCommsRobotRoutes(distance_matrices_with_traversability, robots, is_sampled,
                                                 time_resolution, max_time_steps, pursuit_cell_ids,
                                                 arrival_probability);
  if (pursuit_cell_ids.empty())
  {
    return;
  }

  // Get the number of robots to pursue
  std::vector<int> pursuit_robot_ids;
  GetInCommsRobotIDs(robots, pursuit_robot_ids);

  std::vector<std::vector<int>> pursuit_distance_matrix;
  ComputePursuitDistanceMatrix(robots, pursuit_robot_ids, pursuit_cell_ids, distance_matrices_with_traversability,
                               time_resolution, pursuit_distance_matrix);

  // Solve the TOPwTVP to get the route
  std::vector<std::vector<int>> TOPwTVR_paths;
  SolveTOPwTVP(pursuit_robot_ids, pursuit_cell_ids, arrival_probability, pursuit_distance_matrix, max_time_steps,
               TOPwTVR_paths);

  // Visualize the paths
  VisualizePursuitPaths(pursuit_robot_ids, pursuit_cell_ids, robots, pursuit_distance_matrix, TOPwTVR_paths);

  // Convert the path to the route in distance_matrices_with_traversability indices
  ConvertToRouteInDMIndices(pursuit_robot_ids, pursuit_cell_ids, TOPwTVR_paths, relay_comms_TOPwTVP_solution);
}

bool GridWorld::SolveVRPAssumeComms(const std::vector<int>& exploring_cell_ids, const std::vector<tare::Robot>& robots,
                                    const tare::VRPCost& no_comms_cost, const RouteInDMIndices& no_comms_vrp_solution,
                                    const DistanceMatrix& distance_matrices_with_traversability,
                                    const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                    RouteInDMIndices& assume_comms_vrp_solution,
                                    RouteInDMIndices& relay_comms_vrp_solution)
{
  assume_comms_vrp_solution.clear();
  relay_comms_vrp_solution.clear();

  int sample_iterations = 3;

  std::vector<int32_t> sampled_numbers;
  int32_t sampled_number;
  RouteInDMIndices other_robot_relay_comms_vrp_solution;
  RouteInDMIndices other_robot_assume_comms_vrp_solution;
  tare::VRPCost other_robot_min_cost =
      GetPeerRobotsAssumeCommsSolution(robots, distance_matrices_with_traversability, sampled_numbers, sampled_number,
                                       other_robot_relay_comms_vrp_solution, other_robot_assume_comms_vrp_solution);

  tare::VRPCost min_cost = no_comms_cost;
  bool found_better_solution = false;

  if (LessCost(other_robot_min_cost, no_comms_cost))
  {
    min_cost = other_robot_min_cost;
    relay_comms_vrp_solution = other_robot_relay_comms_vrp_solution;
    assume_comms_vrp_solution = other_robot_assume_comms_vrp_solution;
    found_better_solution = true;

    SampleRobotIntToVector(sampled_number, is_sampled_last_iteration_);
    SampleRobotIntToRobotIDs(sampled_number, relay_comms_robot_ids_);

    // ROS_INFO_STREAM(misc_utils_ns::ColoredText("Using peer's relay comms solution", misc_utils_ns::TextColor::BLUE));
    // ROS_INFO_STREAM("other robot min cost: " << other_robot_min_cost);
  }

  // Sample out-of-comms robot nodes
  std::vector<std::vector<bool>> is_sampled;
  bool has_sampled_robot_indices = SampleOutOfCommsRobotIndices(robots, sample_iterations, is_sampled);
  if (!has_sampled_robot_indices)
  {
    return false;
  }

  std::vector<tare::Robot> in_comms_robots;
  for (const auto& robot : robots)
  {
    if (robot.in_comms_)
    {
      in_comms_robots.push_back(robot);
    }
  }

  for (int itr = 0; itr < is_sampled.size(); itr++)
  {
    std::stringstream sampled_robot_stream;
    for (int robot_id = 0; robot_id < is_sampled[itr].size(); robot_id++)
    {
      if (is_sampled[itr][robot_id])
      {
        sampled_robot_stream << robot_id << " ";
      }
    }

    RouteInDMIndices relay_comms_vrp_solution_itr(robots.size(), std::vector<int>());

    // relay_comms_vrp_solution_itr is in dm index space, with the 1st dimension = # robots
    ComputePursuitRoute(distance_matrices_with_traversability, robots, is_sampled[itr], relay_comms_vrp_solution_itr);

    bool found_relay_plan = false;
    for (int i = 0; i < relay_comms_vrp_solution_itr.size(); i++)
    {
      if (robots[i].in_comms_ && relay_comms_vrp_solution_itr[i].size() > 2)
      {
        found_relay_plan = true;
        break;
      }
    }
    if (!found_relay_plan)
    {
      continue;
    }

    std::vector<tare::Robot> sampled_comms_robots = robots;
    for (int robot_id = 0; robot_id < is_sampled[itr].size(); robot_id++)
    {
      if (is_sampled[itr][robot_id])
      {
        if (robot_id == kRobotID || robots[robot_id].in_comms_ || robots[robot_id].lost_)
        {
          ROS_ERROR_STREAM("Invalid sampled robot_id: " << robot_id << " in comms: " << robots[robot_id].in_comms_
                                                        << " lost: " << robots[robot_id].lost_);
          continue;
        }
        sampled_comms_robots[robot_id].in_comms_ = true;
      }
    }

    // Solve the VRP problem assumes comms
    DistanceMatrix distance_matrices_assumes_comms;
    GetDistanceMatricesNoComms(exploring_cell_ids, sampled_comms_robots, distance_matrices_with_traversability,
                               distance_matrices_assumes_comms);

    RouteInDMIndices assume_comms_initial_route;
    GetInitialRoutesFromVRPSolution(exploring_cell_ids, distance_matrices_assumes_comms, no_comms_vrp_solution,
                                    assume_comms_initial_route);

    RouteInDMIndices assume_comms_vrp_solution_itr;  // Indices w.r.t. distance matrix
    tare::VRPCost with_comms_cost =
        SolveVRP(distance_matrices_assumes_comms, assume_comms_initial_route, assume_comms_vrp_solution_itr);

    tare::VRPCost assume_comms_cost = CombineCommsRelayCost(relay_comms_vrp_solution_itr, assume_comms_vrp_solution_itr,
                                                            distance_matrices_assumes_comms, true);

    if (LessCost(assume_comms_cost, min_cost))
    {
      min_cost = assume_comms_cost;
      assume_comms_vrp_solution = assume_comms_vrp_solution_itr;
      relay_comms_vrp_solution = relay_comms_vrp_solution_itr;
      is_sampled_last_iteration_ = is_sampled[itr];

      found_better_solution = true;
      relay_comms_robot_ids_.clear();
      for (int robot_id = 0; robot_id < is_sampled[itr].size(); robot_id++)
      {
        if (is_sampled[itr][robot_id])
        {
          relay_comms_robot_ids_.push_back(robot_id);
        }
      }
    }
  }

  if (found_better_solution && !relay_comms_vrp_solution.empty() && relay_comms_vrp_solution[kRobotID].size() > 2)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void GridWorld::VRPSolutionToOrderedCellIDs(const std::vector<tare::Robot>& robots,
                                            const std::vector<int>& exploring_cell_ids,
                                            const RouteInDMIndices& vrp_solution, RouteInCellIndices& ordered_cell_ids)
{
  int robot_num = robots.size();
  ordered_cell_ids.clear();
  for (int i = 0; i < vrp_solution.size(); i++)
  {
    std::vector<int> cell_ids;
    for (int j = 0; j < vrp_solution[i].size(); j++)
    {
      int node_index = vrp_solution[i][j];
      if (node_index >= 0 && node_index < robot_num)
      {
        int cell_id = GetCellInd(robots[node_index].in_comms_position_);
        cell_ids.push_back(cell_id);
      }
      else
      {
        int cell_id_index = vrp_solution[i][j] - robot_num;
        int cell_id = exploring_cell_ids[cell_id_index];
        cell_ids.push_back(cell_id);
      }
    }
    ordered_cell_ids.push_back(cell_ids);
  }
}

tare_msgs::Cell GridWorld::ToCellMsg(int cell_ind, CellStatus cell_status)
{
  tare_msgs::Cell cell;
  cell.id = cell_ind;
  cell.known_by_robot_ids = GetCellSyncedRobotIDs(cell_ind);
  cell.mobility_type = static_cast<int>(GetCellMobilityType(cell_ind));
  cell.status = static_cast<int>(cell_status);
  cell.update_id = GetCellUpdateID(cell_ind);
  return cell;
}

int GridWorld::GetLongestRouteDistance(const DistanceMatrix& distance_matrices, const RouteInDMIndices& vrp_solution,
                                       const std::vector<bool>& is_sampled,
                                       const std::vector<int>& sampled_node_indices)
{
  std::vector<int> distance_to_sampled_node(sampled_node_indices.size(), 0);

  for (int robot_id = 0; robot_id < vrp_solution.size(); robot_id++)
  {
    int route_distance = 0;
    if (!is_sampled[robot_id])
    {
      for (int j = 1; j < vrp_solution[robot_id].size(); j++)
      {
        route_distance += distance_matrices[robot_id][j - 1][j];
        for (int k = 0; k < sampled_node_indices.size(); k++)
        {
          if (vrp_solution[robot_id][j] == sampled_node_indices[k])
          {
            distance_to_sampled_node[k] = route_distance;
            break;
          }
        }
      }
    }
  }

  std::vector<int> route_distances(vrp_solution.size(), 0);
  for (int i = 0; i < vrp_solution.size(); i++)
  {
    int route_distance = 0;
    for (int j = 1; j < vrp_solution[i].size(); j++)
    {
      int prev_index = vrp_solution[i][j - 1];
      int curr_index = vrp_solution[i][j];
      route_distance += distance_matrices[i][prev_index][curr_index];
    }
    route_distances[i] = route_distance;
  }
  for (int i = 0; i < distance_to_sampled_node.size(); i++)
  {
    route_distances[i] += distance_to_sampled_node[i];
  }

  int longest_route_distance = 0;
  for (int i = 0; i < route_distances.size(); i++)
  {
    if (longest_route_distance < route_distances[i])
    {
      longest_route_distance = route_distances[i];
    }
  }

  return longest_route_distance;
}

void GridWorld::PrintVRPSolution(const RouteInDMIndices& vrp_solution)
{
  bool is_initial_guess = false;
  for (int i = 0; i < vrp_solution.size(); i++)
  {
    if (vrp_solution[i].empty())
    {
      is_initial_guess = true;
      break;
    }
    else if (vrp_solution[i][0] != i)
    {
      is_initial_guess = true;
      break;
    }
  }

  for (int i = 0; i < vrp_solution.size(); i++)
  {
    if (is_initial_guess)
    {
      if (vrp_solution[i].empty())
      {
        continue;
      }
    }
    else
    {
      if (vrp_solution[i].size() <= 2)
      {
        continue;
      }
    }
  }
}

void GridWorld::CheckLostRobot(std::vector<tare::Robot>& robots)
{
  if (global_exploring_cell_indices_.empty())
  {
    for (int i = 0; i < robots.size(); i++)
    {
      if (i == kRobotID || robots[i].in_comms_)
      {
        robots[i].lost_ = false;
      }
      else
      {
        robots[i].lost_ = true;
      }
    }
    return;
  }

  for (int i = 0; i < robots.size(); i++)
  {
    if (i == kRobotID || robots[i].in_comms_)
    {
      robots[i].lost_ = false;
      continue;
    }

    bool no_exploring_cells = true;
    for (int j = 0; j < global_exploring_cell_indices_.size(); j++)
    {
      int cell_id = global_exploring_cell_indices_[j];
      if (IsCellSyncedByRobot(cell_id, i) && !IsCellVisitedForRelayComms(cell_id, i))
      {
        no_exploring_cells = false;
        break;
      }
    }

    bool robot_cell_visited = false;
    int robot_cell_id = GetCellInd(robots[i].in_comms_position_);
    CellStatus cell_status = GetCellStatus(robot_cell_id);
    if (IsCellVisitedForRelayComms(robot_cell_id, i) || cell_status == CellStatus::COVERED ||
        cell_status == CellStatus::COVERED_BY_OTHERS || IsCellCoveredToOthers(robot_cell_id))
    {
      robot_cell_visited = true;
    }

    if (no_exploring_cells && robot_cell_visited)
    {
      robots[i].lost_ = true;
    }
    else
    {
      robots[i].lost_ = false;
    }
  }
}

bool GridWorld::HasKnowledgeToShare(const std::vector<tare::Robot>& robots)
{
  int out_of_comms_robot_num = 0;
  for (const auto& robot : robots)
  {
    if (!robot.in_comms_ && !robot.lost_)
    {
      out_of_comms_robot_num++;
    }
  }

  if (out_of_comms_robot_num == 0)
  {
    return false;
  }

  std::vector<int> new_knowledge_cell_ids;
  std::vector<int> new_knowledge_cell_index;
  std::vector<int> new_knowledge_cell_unknown_count;
  int index = 0;
  bool has_to_share = false;
  for (const auto& cell_id : global_exploring_cell_indices_)
  {
    int unknown_count = 0;
    for (int i = 0; i < robots.size(); i++)
    {
      int robot_id = robots[i].id_;
      if (robot_id == kRobotID)
      {
        continue;
      }
      if (!robots[i].in_comms_ && !robots[i].lost_ && !IsCellSyncedByRobot(cell_id, robot_id))
      {
        // Number of robots that are out of comms, but not lost, don't know
        // about the cell
        unknown_count++;
      }
    }
    if (unknown_count > 0)
    {
      has_to_share = true;
      new_knowledge_cell_ids.push_back(cell_id);
      new_knowledge_cell_index.push_back(index + robots.size());
      new_knowledge_cell_unknown_count.push_back(unknown_count);
    }
    index++;
  }

  return has_to_share;
}

void GridWorld::GetRobotsInComms(const std::vector<tare::Robot>& robots, std::vector<tare::Robot>& robots_in_comms)
{
  robots_in_comms.clear();
  for (const auto& robot : robots)
  {
    if (robot.in_comms_)
    {
      robots_in_comms.push_back(robot);
    }
  }
}

tare::VRPCost GridWorld::GetInitialRoute(const tare::VRPPlan& vrp_plan, const std::vector<int>& exploring_cell_ids,
                                         const DistanceMatrix& distance_matrices, RouteInDMIndices& initial_route)
{
  initial_route.clear();
  if (vrp_plan.no_comms_ordered_cell_ids_.empty())
  {
    return tare::VRPCost(INT_MAX, misc_utils_ns::INF_DISTANCE);
  }

  RouteInDMIndices vrp_solution;
  RouteInCellIndicesToDistanceMatricesIndices(vrp_plan.no_comms_ordered_cell_ids_, vrp_solution);

  GetInitialRoutesFromVRPSolution(exploring_cell_ids, distance_matrices, vrp_solution, initial_route);

  tare::VRPCost vrp_cost;
  std::vector<int> per_route_length;

  ComputeVRPSolutionCost(initial_route, distance_matrices, vrp_cost.dropped_nodes_number_,
                         vrp_cost.longest_route_length_, per_route_length);
  return vrp_cost;
}

void GridWorld::GetPeerRobotExploringCellIDs(const std::vector<tare::Robot>& robots,
                                             std::vector<int>& peer_robot_local_exploring_cell_ids)
{
  peer_robot_local_exploring_cell_ids.clear();
  for (int i = 0; i < robots.size(); i++)
  {
    if (robots[i].id_ == kRobotID || !robots[i].in_comms_)
    {
      continue;
    }
    for (const auto& cell_id : robots[i].vrp_plan_.local_exploring_cell_ids_)
    {
      peer_robot_local_exploring_cell_ids.push_back(cell_id);
    }
  }
  misc_utils_ns::UniquifyIntVector(peer_robot_local_exploring_cell_ids);
}

void GridWorld::AddNodeToVRPPlan(int to_add_node_index, const DistanceMatrix& distance_matrices,
                                 RouteInDMIndices& min_cost_routes)
{
  int robot_num = min_cost_routes.size();
  int min_cost_robot_id = -1;
  int min_cost_node_id = -1;
  int min_added_distance = misc_utils_ns::INF_DISTANCE;
  for (int robot_id = 0; robot_id < robot_num; robot_id++)
  {
    for (int node_id = 0; node_id < min_cost_routes[robot_id].size() - 1; node_id++)
    {
      int from_node_index = min_cost_routes[robot_id][node_id];
      int to_node_index = min_cost_routes[robot_id][node_id + 1];

      int added_distance = distance_matrices[robot_id][from_node_index][to_add_node_index] +
                           distance_matrices[robot_id][to_add_node_index][to_node_index] -
                           distance_matrices[robot_id][from_node_index][to_node_index];

      if (added_distance < min_added_distance)
      {
        min_added_distance = added_distance;
        min_cost_robot_id = robot_id;
        min_cost_node_id = node_id + 1;
      }
    }
  }
  if (min_cost_robot_id >= 0 && min_cost_robot_id < robot_num && min_cost_node_id >= 0 &&
      min_cost_node_id < min_cost_routes[min_cost_robot_id].size())
  {
    std::vector<int>::iterator it = min_cost_routes[min_cost_robot_id].begin();
    min_cost_routes[min_cost_robot_id].insert(it + min_cost_node_id, to_add_node_index);
  }
}

void GridWorld::RouteInCellIndicesToDistanceMatricesIndices(const RouteInCellIndices& ordered_cell_ids,
                                                            RouteInDMIndices& routes)
{
  routes.clear();
  for (int i = 0; i < ordered_cell_ids.size(); i++)
  {
    std::vector<int> route;
    route.push_back(i);
    if (ordered_cell_ids[i].size() > 2)
    {
      for (int j = 1; j < ordered_cell_ids[i].size() - 1; j++)
      {
        int cell_id = ordered_cell_ids[i][j];
        if (misc_utils_ns::ElementExistsInVector<int>(global_exploring_cell_indices_, cell_id))
        {
          int dm_index = cell_id_to_dm_index_[cell_id];
          route.push_back(dm_index);
        }
      }
    }
    route.push_back(i);
    routes.push_back(route);
  }
}

void GridWorld::PrintVRPSolutionStats(const std::string& route_name, const RouteInDMIndices& vrp_solution,
                                      const DistanceMatrix& distance_matrices, bool no_cell_to_cell_distance_offset)
{
  ROS_INFO_STREAM(misc_utils_ns::ColoredText(route_name, misc_utils_ns::TextColor::CYAN));
  PrintVRPSolution(vrp_solution);
  int dropped_node_num = 0;
  int max_route_length = 0;
  std::vector<int> per_route_length;
  int total_cost = ComputeVRPSolutionCost(vrp_solution, distance_matrices, dropped_node_num, max_route_length,
                                          per_route_length, no_cell_to_cell_distance_offset);
  std::string dropped_num_str =
      dropped_node_num == 0 ?
          misc_utils_ns::ColoredText(std::to_string(dropped_node_num), misc_utils_ns::TextColor::WHITE) :
          misc_utils_ns::ColoredText(std::to_string(dropped_node_num), misc_utils_ns::TextColor::RED);
  std::string max_route_length_str =
      max_route_length >= misc_utils_ns::INF_DISTANCE ?
          misc_utils_ns::ColoredText(std::to_string(max_route_length), misc_utils_ns::TextColor::RED) :
          misc_utils_ns::ColoredText(std::to_string(max_route_length), misc_utils_ns::TextColor::WHITE);
  ROS_INFO_STREAM("Cost: dropped " << dropped_num_str << " max_length: " << max_route_length_str);
  ROS_INFO_STREAM("Per route length: ");
  misc_utils_ns::PrintVector<int>(per_route_length);
}

void GridWorld::CheckNotVisitedCells(const RouteInCellIndices& global_plan)
{
  std::vector<bool> visited(global_exploring_cell_indices_.size(), false);
  for (int i = 0; i < global_plan.size(); i++)
  {
    for (int j = 0; j < global_plan[i].size(); j++)
    {
      int cell_id = global_plan[i][j];
      for (int k = 0; k < global_exploring_cell_indices_.size(); k++)
      {
        if (global_exploring_cell_indices_[k] == cell_id)
        {
          visited[k] = true;
          break;
        }
      }
    }
  }
}

tare::VRPCost GridWorld::GetPeerRobotsAssumeCommsSolution(const std::vector<tare::Robot>& robots,
                                                          const DistanceMatrix& distance_matrices,
                                                          std::vector<int32_t>& sampled_numbers,
                                                          int32_t& sampled_number,
                                                          RouteInDMIndices& other_robot_relay_comms_vrp_solution,
                                                          RouteInDMIndices& other_robot_assume_comms_vrp_solution)
{
  sampled_numbers.clear();
  tare::VRPCost min_cost(INT_MAX, misc_utils_ns::INF_DISTANCE);
  for (int robot_id = 0; robot_id < robots.size(); robot_id++)
  {
    if (!robots[robot_id].in_comms_)
    {
      continue;
    }
    RouteInDMIndices robot_relay_comms_vrp_solution;
    RouteInDMIndices robot_assume_comms_vrp_solution;
    int32_t sampled_number_robot = 0;
    tare::VRPCost robot_cost =
        GetRobotAssumeCommsSolution(robots, robots[robot_id].vrp_plan_, distance_matrices, sampled_number_robot,
                                    robot_relay_comms_vrp_solution, robot_assume_comms_vrp_solution);

    if (LessCost(robot_cost, min_cost))
    {
      min_cost = robot_cost;
      other_robot_relay_comms_vrp_solution = robot_relay_comms_vrp_solution;
      other_robot_assume_comms_vrp_solution = robot_assume_comms_vrp_solution;
      sampled_number = sampled_number_robot;
    }
    if (sampled_number != 0 && !misc_utils_ns::ElementExistsInVector<int32_t>(sampled_numbers, sampled_number))
    {
      sampled_numbers.push_back(sampled_number);
    }
  }
  return min_cost;
}

tare::VRPCost GridWorld::GetRobotAssumeCommsSolution(const std::vector<tare::Robot>& robots,
                                                     const tare::VRPPlan vrp_plan,
                                                     const DistanceMatrix& distance_matrices, int32_t& sampled_number,
                                                     RouteInDMIndices& robot_relay_comms_vrp_solution,
                                                     RouteInDMIndices& robot_assume_comms_vrp_solution)
{
  tare::VRPCost cost(INT_MAX, misc_utils_ns::INF_DISTANCE);
  if (vrp_plan.sampled_robots_ == 0 || vrp_plan.relay_comms_ordered_cell_ids_.empty() ||
      vrp_plan.assume_comms_ordered_cell_ids_.empty())
  {
    return cost;
  }
  sampled_number = vrp_plan.sampled_robots_;

  std::vector<int> relay_robot_ids;
  SampleRobotIntToRobotIDs(sampled_number, relay_robot_ids);
  bool relay_plan_valid = OrderedCellIDsToRelayCommsVRPSolution(vrp_plan.relay_comms_ordered_cell_ids_, sampled_number,
                                                                robots, robot_relay_comms_vrp_solution);

  if (relay_plan_valid)
  {
    OrderedCellIDsToAssumeCommsVRPSolution(vrp_plan.assume_comms_ordered_cell_ids_, robot_assume_comms_vrp_solution);
  }

  if (relay_plan_valid)
  {
    return CombineCommsRelayCost(robot_relay_comms_vrp_solution, robot_assume_comms_vrp_solution, distance_matrices,
                                 true);
  }
  else
  {
    return cost;
  }
}

int32_t GridWorld::SampleRobotVectorToInt(const std::vector<bool>& sample_robot_vector)
{
  int32_t sample_robot_int = 0;
  for (int i = 0; i < kRobotNum; i++)
  {
    if (sample_robot_vector[i])
    {
      sample_robot_int += (1 << i);
    }
  }
  return sample_robot_int;
}

void GridWorld::SampleRobotIntToVector(int32_t sample_robot_int, std::vector<bool>& sample_robot_vector)
{
  sample_robot_vector.resize(kRobotNum, false);
  for (int i = 0; i < kRobotNum; i++)
  {
    if (sample_robot_int & (1 << i))
    {
      sample_robot_vector[i] = true;
    }
  }
}

void GridWorld::SampleRobotIntToRobotIDs(int32_t sample_robot_int, std::vector<int>& robot_ids)
{
  std::vector<bool> sample_robot_vector;
  SampleRobotIntToVector(sample_robot_int, sample_robot_vector);
  robot_ids.clear();
  for (int robot_id = 0; robot_id < sample_robot_vector.size(); robot_id++)
  {
    if (sample_robot_vector[robot_id])
    {
      robot_ids.push_back(robot_id);
    }
  }
}

bool GridWorld::OrderedCellIDsToRelayCommsVRPSolution(const RouteInCellIndices& ordered_cell_ids,
                                                      int32_t sampled_number, const std::vector<tare::Robot>& robots,
                                                      RouteInDMIndices& relay_comms_vrp_solution)
{
  std::vector<int> sampled_robot_ids;
  SampleRobotIntToRobotIDs(sampled_number, sampled_robot_ids);

  // Check if the robot is lost, if so, nullify the plan
  for (const auto& sampled_robot_id : sampled_robot_ids)
  {
    if (robots[sampled_robot_id].lost_)
    {
      return false;
    }
  }

  relay_comms_vrp_solution.clear();
  for (int robot_id = 0; robot_id < ordered_cell_ids.size(); robot_id++)
  {
    std::vector<int> route;
    route.push_back(robot_id);
    if (ordered_cell_ids[robot_id].size() > 2)
    {
      for (int node_id = 1; node_id < ordered_cell_ids[robot_id].size() - 1; node_id++)
      {
        int cell_id = ordered_cell_ids[robot_id][node_id];
        if (misc_utils_ns::ElementExistsInVector<int>(global_exploring_cell_indices_, cell_id))
        {
          // Check visited
          bool visited = false;
          for (const auto& sampled_robot_id : sampled_robot_ids)
          {
            if (IsCellVisitedForRelayComms(cell_id, sampled_robot_id))
            {
              visited = true;
              break;
            }
          }
          if (!visited)
          {
            // Remove duplicates
            int dm_index = cell_id_to_dm_index_[cell_id];
            if (dm_index != route.back())
            {
              route.push_back(cell_id_to_dm_index_[cell_id]);
            }
          }
        }
        else
        {
          for (const auto& sampled_robot_id : sampled_robot_ids)
          {
            int robot_cell_id = GetCellInd(robots[sampled_robot_id].in_comms_position_);
            if (cell_id == robot_cell_id)
            {
              if (sampled_robot_id != route.back())
              {
                route.push_back(sampled_robot_id);
              }
              break;
            }
          }
        }
      }
    }

    route.push_back(robot_id);
    relay_comms_vrp_solution.push_back(route);
  }

  bool relay_plan_valid = false;
  for (int i = 0; i < relay_comms_vrp_solution.size(); i++)
  {
    if (relay_comms_vrp_solution[i].size() > 2)
    {
      relay_plan_valid = true;
      break;
    }
  }

  return relay_plan_valid;
}

void GridWorld::OrderedCellIDsToAssumeCommsVRPSolution(const RouteInCellIndices& ordered_cell_ids,
                                                       RouteInDMIndices& assume_comms_vrp_solution)
{
  assume_comms_vrp_solution.clear();
  for (int robot_id = 0; robot_id < ordered_cell_ids.size(); robot_id++)
  {
    std::vector<int> route;
    route.push_back(robot_id);
    if (ordered_cell_ids[robot_id].size() > 2)
    {
      for (int node_id = 1; node_id < ordered_cell_ids[robot_id].size() - 1; node_id++)
      {
        int cell_id = ordered_cell_ids[robot_id][node_id];
        if (misc_utils_ns::ElementExistsInVector<int>(global_exploring_cell_indices_, cell_id))
        {
          route.push_back(cell_id_to_dm_index_[cell_id]);
        }
      }
    }
    route.push_back(robot_id);
    assume_comms_vrp_solution.push_back(route);
  }
}

bool GridWorld::PlanForRendezvous(const std::vector<tare::Robot>& robots, const DistanceMatrix& distance_matrices,
                                  std::vector<int>& rendezvous_global_plan)
{
  nav_msgs::Path path_to_rendezvous;
  std::vector<int> path_to_rendezvous_cell_ids;
  if (!IsConnectedOnRoadmap(cur_robot_cell_ind_))
  {
    return false;
  }
  double path_length = roadmap_.GetShortestPath(cur_robot_cell_ind_, rendezvous_manager_.GetCurrentRendezvousCellID(),
                                                false, path_to_rendezvous, path_to_rendezvous_cell_ids);
  int time_to_rendezvous = path_length / 2.0;

  wait_ = false;
  if (rendezvous_manager_.TimeToMeet(time_to_rendezvous) || go_to_rendezvous_)
  {
    if (AllRobotsInComms(robots))
    {
      rendezvous_manager_.PlanForNextRendezvous(robots, kRobotID, global_exploring_cell_indices_, distance_matrices,
                                                kCellToCellDistanceOffset);
      if (AllRobotsSyncedWithRendezvous(robots))
      {
        rendezvous_manager_.UpdateCurrentRendezvousPlan();
        if (!wait_at_rendezvous_)
        {
          wait_at_rendezvous_ = true;
          wait_at_rendezvous_count_ = 0;
        }
        wait_at_rendezvous_count_++;
        if (wait_at_rendezvous_count_ >= 5)
        {
          wait_at_rendezvous_ = false;
          return false;
        }
        else
        {
          wait_ = true;
          rendezvous_global_plan_.clear();
          rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
          rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
          return true;
        }
      }
      else
      {
        wait_ = true;
        rendezvous_global_plan_.clear();
        rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
        rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
        return true;
      }
    }
    else
    {
      int robot_cell_id = GetCellInd(robot_position_);
      int rendezvous_cell_id = rendezvous_manager_.GetCurrentRendezvousCellID();
      if (!ConnectedToRendezvous(robots, rendezvous_cell_id))
      {
        rendezvous_global_plan_.clear();
        rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
        rendezvous_global_plan_.push_back(rendezvous_manager_.GetCurrentRendezvousCellID());
        rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
        return true;
      }
      else
      {
        wait_ = true;
        rendezvous_global_plan_.clear();
        rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
        rendezvous_global_plan_.push_back(cur_robot_cell_ind_);
        return true;
      }
    }
  }
  else
  {
    if (AllRobotsInComms(robots))
    {
      rendezvous_manager_.ResetTimer();
      rendezvous_manager_.SyncNextRendezvous(robots, kRobotID);
      if (AllRobotsSyncedWithRendezvous(robots))
      {
        rendezvous_manager_.UpdateCurrentRendezvousPlan();
      }
    }
    return false;
  }
}

bool GridWorld::AllRobotsInComms(const std::vector<tare::Robot>& robots)
{
  for (const auto& robot : robots)
  {
    if (!robot.in_comms_)
    {
      return false;
    }
  }
  return true;
}

bool GridWorld::AllRobotsSyncedWithRendezvous(const std::vector<tare::Robot>& robots)
{
  if (!kRendezvous)
  {
    return false;
  }

  int self_rendezvous_cell_id = rendezvous_manager_.GetNextRendezvousCellID();
  int self_rendezvous_time_interval = rendezvous_manager_.GetNextRendezvousTimeInterval();
  bool synced = true;
  std::stringstream robot_rendezvous_cell_id_stream;
  for (const auto& robot : robots)
  {
    if (robot.vrp_plan_.relay_comms_ordered_cell_ids_.empty())
    {
      synced = false;
      robot_rendezvous_cell_id_stream << " * ";
      continue;
    }
    if (robot.vrp_plan_.relay_comms_ordered_cell_ids_.front().size() < 2)
    {
      synced = false;
      robot_rendezvous_cell_id_stream << " * ";
      continue;
    }

    int robot_rendezvous_cell_id = robot.vrp_plan_.relay_comms_ordered_cell_ids_.front()[0];
    int robot_rendezvous_time_interval = robot.vrp_plan_.relay_comms_ordered_cell_ids_.front()[1];
    robot_rendezvous_cell_id_stream << robot_rendezvous_cell_id << "(" << robot_rendezvous_time_interval << ") ";
    if (robot_rendezvous_cell_id != self_rendezvous_cell_id ||
        robot_rendezvous_time_interval != self_rendezvous_time_interval)
    {
      synced = false;
      continue;
    }
  }

  if (synced)
  {
    rendezvous_manager_.ResetPlannedNextRendezvous();
  }

  return synced;
}

bool GridWorld::ConnectedToRendezvous(const std::vector<tare::Robot>& robots, int rendezvous_cell_id)
{
  int robot_cell_id = GetCellInd(robot_position_);
  if (robot_cell_id == rendezvous_cell_id)
  {
    return true;
  }

  if (!kRendezvousTree)
  {
    return false;
  }

  geometry_msgs::Point rendezvous_cell_position = GetCellPosition(rendezvous_cell_id);
  for (int i = 0; i < robots.size(); i++)
  {
    if (!robots[i].in_comms_ || i == kRobotID)
    {
      continue;
    }
    geometry_msgs::Point robot_position = robots[i].in_comms_position_;
    double distance_to_rendezvous = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
        rendezvous_cell_position, robot_position);
    if (distance_to_rendezvous < kCommsRange / 2)
    {
      return true;
    }
  }
  return false;
}

void GridWorld::SetCommsConfig()
{
  if (kTestID[0] == '0')
  {
    // full comms
    kCommsRange = 1000.0;
    kRelayComms = false;
    kRendezvous = false;
    kRendezvousTree = false;
    kUseTimeBudget = false;
    ROS_INFO_STREAM("Comms config: full comms");
  }
  else
  {
    if (kTestID.size() == 4)
    {
      kCommsRange = 30.0;
    }
    else if (kTestID.size() == 6)
    {
      std::string comms_range_str = kTestID.substr(4, 2);
      kCommsRange = stoi(comms_range_str);
    }
    else if (kTestID.size() == 7)
    {
      std::string comms_range_str = kTestID.substr(4, 3);
      kCommsRange = stoi(comms_range_str);
    }
    else
    {
      ROS_ERROR_STREAM("kTestID wrong size: " << kTestID.size());
    }
    ROS_INFO_STREAM("Comms Range: " << kCommsRange);
    if (kTestID[0] == '1')
    {
      // Ours
      kRelayComms = true;
      kRendezvous = false;
      kRendezvousTree = false;
      ROS_INFO_STREAM("Comms config: relay comms");
    }
    else if (kTestID[0] == '2')
    {
      // Rendezvous tree
      kRelayComms = false;
      kRendezvous = true;
      kRendezvousTree = true;
      ROS_INFO_STREAM("Comms config: rendezvous tree");
    }
    else if (kTestID[0] == '3')
    {
      // Rendezvous middle
      kRelayComms = false;
      kRendezvous = true;
      kRendezvousTree = false;
      kRendezvousType = 0;
      ROS_INFO_STREAM("Comms config: rendezvous middle");
    }
    else if (kTestID[0] == '4')
    {
      // no comms
      kRelayComms = false;
      kRendezvous = false;
      kRendezvousTree = false;
      ROS_INFO_STREAM("Comms config: no comms");
    }
    else if (kTestID[0] == '5')
    {
      // nearest rendezvous point
      kRelayComms = false;
      kRendezvous = true;
      kRendezvousTree = false;
      kRendezvousType = 1;
      ROS_INFO_STREAM("Comms config: nearest rendezvous");
    }
    else if (kTestID[0] == '6')
    {
      // farthest rendezvous point
      kRelayComms = false;
      kRendezvous = true;
      kRendezvousTree = false;
      kRendezvousType = 2;
      ROS_INFO_STREAM("Comms config: farthest rendezvous");
    }
  }
}

}  // namespace grid_world_ns