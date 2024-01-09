/**
 * @file multi_robot_exploration_manager.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that maintains the exploration information from other robots
 * @version 0.1
 * @date 2021-12-26
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <coordination/multi_robot_exploration_manager.h>
#include <utils/misc_utils.h>
#include <grid_world/grid_world.h>

namespace tare
{
RobotManager::RobotManager(int id, MobilityType mobility_type)
  : robot_(id, mobility_type)
  , exploration_info_updated_(false)
  , position_updated_(false)
  , info_update_time_(ros::Time::now())
  , exploration_info_count_(0)
{
  name_ = tare::MobilityTypeToString(mobility_type) + std::to_string(id);
  in_range_robot_ids_ = (1 << id);
}

void RobotManager::InitializeSubscribers(ros::NodeHandle& nh)
{
  std::string exploration_info_topic = "/" + name_ + "/exploration_info";
  exploration_info_sub_ = nh.subscribe(exploration_info_topic, 1, &RobotManager::ExplorationInfoCallback, this);
  std::string position_topic = "/" + name_ + "/position";
  position_sub_ = nh.subscribe(position_topic, 1, &RobotManager::PositionCallback, this);
  std::cout << "Initialized robot " << name_ << std::endl;
  std::cout << "Subscribing to: " << std::endl;
  std::cout << exploration_info_topic << std::endl;
  std::cout << position_topic << std::endl;
}

void RobotManager::ExplorationInfoCallback(const tare_msgs::ExplorationInfo::ConstPtr& exploration_info)
{
  double update_from_last_sec = (ros::Time::now() - info_update_time_).toSec();
  info_update_time_ = ros::Time::now();
  // Just for sim
  if (!exploration_info->robot_update_secs.empty())
  {
    in_range_robot_ids_ = exploration_info->robot_update_secs.front();
  }

  if (!robot_.in_comms_)
  {
    return;
  }
  exploration_info_ = *exploration_info;

  std::vector<int32_t> msg;
  for (const auto& e : exploration_info_.global_cell_ids)
  {
    msg.push_back(e);
  }

  robot_.secs_from_start_ = exploration_info_.secs_from_start;

  tare::DecodeGlobalVRPPlan(msg, robot_.vrp_plan_);

  // std::cout << "*************************************" << std::endl;
  // std::cout << "robot " << robot_.id_
  //           << " no comms vrp plan size: " << robot_.vrp_plan_.no_comms_ordered_cell_ids_.size() << std::endl;

  robot_.vrp_plan_.local_exploring_cell_ids_.clear();
  for (const auto& e : exploration_info_.local_cell_ids)
  {
    robot_.vrp_plan_.local_exploring_cell_ids_.push_back(e);
  }

  robot_update_secs_.clear();
  if (!exploration_info_.robot_update_secs.empty())
  {
    for (int i = 1; i < exploration_info_.robot_update_secs.size(); i++)
    {
      robot_update_secs_.push_back(exploration_info_.robot_update_secs[i]);
    }
  }

  robot_positions_.clear();
  for (const auto& e : exploration_info_.robot_positions)
  {
    geometry_msgs::Point point;
    point.x = e.x;
    point.y = e.y;
    point.z = e.z;
    robot_positions_.push_back(point);
  }

  robot_.finished_exploration_ = false;
  if (robot_.vrp_plan_.local_exploring_cell_ids_.empty())
  {
    if (robot_.vrp_plan_.no_comms_ordered_cell_ids_.empty())
    {
      robot_.finished_exploration_ = true;
    }
    else if (robot_.vrp_plan_.no_comms_ordered_cell_ids_[robot_.id_].size() <= 2)
    {
      robot_.finished_exploration_ = true;
    }
  }

  robot_.visited_for_convoy_ = false;

  exploration_info_updated_ = true;
  exploration_info_count_++;
}

void RobotManager::PositionCallback(const geometry_msgs::PointStamped::ConstPtr& position)
{
  // info_update_time_ = ros::Time::now();
  robot_.position_ = position->point;
  if (!robot_.in_comms_)
  {
    return;
  }
  robot_.in_comms_position_ = robot_.position_;
  position_updated_ = true;
}

void RobotManager::GetCells(std::vector<tare_msgs::Cell>& cells) const
{
  cells.clear();
  for (const auto& cell : exploration_info_.cells)
  {
    cells.push_back(cell);
  }
}

void RobotManager::GetExploringCellIDs(std::vector<int>& exploring_cell_ids) const
{
  exploring_cell_ids.clear();
  if (robot_.vrp_plan_.no_comms_ordered_cell_ids_.empty())
  {
    return;
  }
  for (int i = 0; i < robot_.vrp_plan_.no_comms_ordered_cell_ids_.size(); i++)
  {
    if (robot_.vrp_plan_.no_comms_ordered_cell_ids_[i].size() > 2)
    {
      for (int j = 1; j < robot_.vrp_plan_.no_comms_ordered_cell_ids_[i].size() - 1; j++)
      {
        exploring_cell_ids.push_back(robot_.vrp_plan_.no_comms_ordered_cell_ids_[i][j]);
      }
    }
  }
}

void RobotManager::GetRoadmapEdges(std::vector<std::pair<int, int>>& roadmap_edges) const
{
  roadmap_edges.clear();
  for (const auto& edge : exploration_info_.roadmap_edges)
  {
    roadmap_edges.emplace_back(edge.from_cell_id, edge.to_cell_id);
  }
}

void RobotManager::GetRoadmapEdges(std::vector<tare_msgs::Edge>& roadmap_edges) const
{
  roadmap_edges.clear();
  for (const auto& edge : exploration_info_.roadmap_edges)
  {
    roadmap_edges.push_back(edge);
  }
}

MultiRobotExplorationManager::MultiRobotExplorationManager(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
  nh_private.getParam("robot_types", robot_types_);
  id_ = misc_utils_ns::getParam<int>(nh_private, "robot_id", 0);
  kRobotNum = misc_utils_ns::getParam<int>(nh_private, "kRobotNum", 1);
  kHeterogeneous = misc_utils_ns::getParam<bool>(nh_private, "kHeterogeneous", 1);
  kCommsRange = misc_utils_ns::getParam<double>(nh_private, "kCommsRange", DBL_MAX);
  kLostCommsSecThr = misc_utils_ns::getParam<int>(nh_private, "kLostCommsSecThr", 10);
  kTestID = misc_utils_ns::getParam<std::string>(nh_private, "kTestID", "0001");

  if (!kHeterogeneous && !robot_types_.empty())
  {
    robot_types_.resize(kRobotNum, robot_types_.front());
  }

  ROS_INFO_STREAM("multirobot manager: kTestID: " << kTestID);
  if (kTestID[0] == '0')
  {
    // full comms
    kCommsRange = 1000.0;
  }
  else
  {
    // kCommsRange = 30.0;
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
  }
  ROS_INFO_STREAM("multirobot manager: kRobotNum: " << kRobotNum);
  ROS_INFO_STREAM("multirobot manager: kCommsRange: " << kCommsRange);

  MY_ASSERT(id_ >= 0 && id_ < robot_types_.size());
  for (int i = 0; i < robot_types_.size(); i++)
  {
    std::string robot_type = robot_types_[i];
    if (tare::StringToMobilityType(robot_type) != MobilityType::NONE &&
        tare::StringToMobilityType(robot_type) != MobilityType::WHEELED &&
        tare::StringToMobilityType(robot_type) != MobilityType::TRACKED &&
        tare::StringToMobilityType(robot_type) != MobilityType::AERIAL)
    {
      ROS_ERROR_STREAM("Wrong robot type: " << robot_type);
      exit(1);
    }
  }

  for (int i = 0; i < robot_types_.size(); i++)
  {
    robots_.push_back(RobotManager(i, tare::StringToMobilityType(robot_types_[i])));
    if (i == id_)
    {
      name_ = robot_types_[id_] + std::to_string(id_);
      type_ = tare::StringToMobilityType(robot_types_[i]);
      robots_[i].SetInComms(true);
    }
  }

  for (int i = 0; i < robots_.size(); i++)
  {
    if (i == id_)
    {
      continue;
    }
    robots_[i].InitializeSubscribers(nh);
  }
}

void MultiRobotExplorationManager::GetRoadmapEdges(std::vector<tare_msgs::Edge>& roadmap_edges) const
{
  // TODO: by robot type
  roadmap_edges.clear();
  std::vector<std::pair<int, int>> added_edges;
  for (const auto& robot : robots_)
  {
    std::vector<tare_msgs::Edge> robot_roadmap_edges;
    robot.GetRoadmapEdges(robot_roadmap_edges);
    for (const auto& edge : robot_roadmap_edges)
    {
      int from_cell_id = edge.from_cell_id;
      int to_cell_id = edge.to_cell_id;
      if (from_cell_id > to_cell_id)
      {
        std::swap(from_cell_id, to_cell_id);
      }
      if (std::find(added_edges.begin(), added_edges.end(), std::make_pair(from_cell_id, to_cell_id)) ==
          added_edges.end())
      {
        roadmap_edges.push_back(edge);
        added_edges.emplace_back(from_cell_id, to_cell_id);
      }
    }
  }
}

void MultiRobotExplorationManager::UpdateCommsStatus(const geometry_msgs::Point& self_position)
{
  robots_[id_].ResetInRangeRobotIDs();
  // std::cout << "dist to robots: " << std::endl;
  int in_comms_robot_count = 0;
  for (auto& robot : robots_)
  {
    if (robot.GetID() == id_)
    {
      in_comms_robot_count++;
      continue;
    }
    int lost_comms_sec = (ros::Time::now() - robot.GetInfoUpdateTime()).toSec();
    if (lost_comms_sec > kLostCommsSecThr)
    {
      robot.SetInComms(false);
    }
    else
    {
      // TODO: Tmp for simulation test
      double dist_to_self =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(robot.GetPosition(), self_position);
      // std::cout << "R" << robot.GetID() << ": " << dist_to_self << " ";
      if (dist_to_self <= kCommsRange)
      {
        robots_[id_].SetRobotInRange(robot.GetID());
        if (robot.IsRobotInRange(id_))
        {
          robot.SetInComms(true);
          in_comms_robot_count++;
        }
      }
      else
      {
        robot.SetInComms(false);
      }
    }
  }
  // std::cout << std::endl;

  int prev_in_comms_robot_count = 0;
  while (prev_in_comms_robot_count != in_comms_robot_count)
  {
    prev_in_comms_robot_count = in_comms_robot_count;
    in_comms_robot_count = 0;
    for (auto& robot : robots_)
    {
      if (robot.GetID() == id_)
      {
        in_comms_robot_count++;
        continue;
      }
      if (!robot.IsInComms())
      {
        continue;
      }

      in_comms_robot_count++;

      for (auto& neighbor_robot : robots_)
      {
        if (neighbor_robot.GetID() == id_ || neighbor_robot.IsInComms())
        {
          continue;
        }
        double dist_to_in_comms_robot = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
            robot.GetPosition(), neighbor_robot.GetPosition());
        if (dist_to_in_comms_robot <= kCommsRange)
        {
          robots_[id_].SetRobotInRange(neighbor_robot.GetID());
          if (neighbor_robot.IsRobotInRange(id_))
          {
            neighbor_robot.SetInComms(true);
            in_comms_robot_count++;
          }
        }
        else
        {
          neighbor_robot.SetInComms(false);
        }
      }
    }
  }
}

bool MultiRobotExplorationManager::IsInComms()
{
  for (const auto& robot : robots_)
  {
    if (robot.GetID() == id_)
    {
      continue;
    }
    if (robot.IsInComms() && (robot.PositionUpdated() || robot.ExplorationInfoUpdated()))
    {
      return true;
    }
  }
  return false;
}

void MultiRobotExplorationManager::UpdateGlobalKnowledge(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world)
{
  for (const auto& robot : robots_)
  {
    if (!robot.ExplorationInfoUpdated() || robot.GetID() == id_)
    {
      continue;
    }
    UpdateCells(robot, grid_world);
    UpdateRoadmap(robot, grid_world);
  }
  SyncGlobalKnowledge(grid_world);

  for (auto& robot : robots_)
  {
    if (robot.GetID() == id_)
    {
      continue;
    }
    robot.SetExplorationInfoUpdated(false);
  }
}

void MultiRobotExplorationManager::UpdateCells(const RobotManager& robot,
                                               const std::unique_ptr<grid_world_ns::GridWorld>& grid_world)
{
  std::vector<tare_msgs::Cell> cells;
  robot.GetCells(cells);
  for (const auto& cell : cells)
  {
    int cell_id = cell.id;
    if (grid_world->IsNeighbor(cell_id) && grid_world->GetCellStatus(cell_id) == grid_world_ns::CellStatus::EXPLORING)
    {
      continue;
    }
    grid_world_ns::CellStatus cell_status = static_cast<grid_world_ns::CellStatus>(cell.status);
    tare::MobilityType mobility_type = static_cast<tare::MobilityType>(cell.mobility_type);
    int update_id = cell.update_id;
    int prev_update_id = grid_world->GetCellUpdateID(cell_id);
    if (update_id > prev_update_id)
    {
      grid_world->SetCellUpdateID(cell_id, update_id);
      grid_world->ResetCellSyncCount(cell_id);
      grid_world->SetCellMobilityType(cell_id, mobility_type);
      grid_world_ns::CellStatus prev_status = grid_world->GetCellStatus(cell_id);
      if ((prev_status == grid_world_ns::CellStatus::UNSEEN ||
           prev_status == grid_world_ns::CellStatus::COVERED_BY_OTHERS) &&
          cell_status == grid_world_ns::CellStatus::EXPLORING)
      {
        grid_world->SetCellStatus(cell_id, grid_world_ns::CellStatus::EXPLORING_BY_OTHERS);
        grid_world->ResetCellSync(cell_id);
      }
      else if ((prev_status == grid_world_ns::CellStatus::EXPLORING ||
                prev_status == grid_world_ns::CellStatus::EXPLORING_BY_OTHERS ||
                prev_status == grid_world_ns::CellStatus::UNSEEN) &&
               cell_status == grid_world_ns::CellStatus::COVERED)
      {
        grid_world->SetCellStatus(cell_id, grid_world_ns::CellStatus::COVERED_BY_OTHERS);
        grid_world->ResetCellSync(cell_id);
      }
    }
  }

  std::vector<int> robot_exploring_cell_ids;
  robot.GetExploringCellIDs(robot_exploring_cell_ids);
  for (const auto& cell_id : robot_exploring_cell_ids)
  {
    grid_world_ns::CellStatus cell_status = grid_world->GetCellStatus(cell_id);
    if (cell_status != grid_world_ns::CellStatus::EXPLORING &&
        cell_status != grid_world_ns::CellStatus::EXPLORING_BY_OTHERS)
    {
      grid_world->ResetCellSyncCount(cell_id);
    }
  }
}

void MultiRobotExplorationManager::UpdateRoadmap(const RobotManager& robot,
                                                 const std::unique_ptr<grid_world_ns::GridWorld>& grid_world)
{
  std::vector<tare_msgs::Edge> roadmap_edges;
  robot.GetRoadmapEdges(roadmap_edges);
  for (const auto& edge : roadmap_edges)
  {
    int from_cell_ind = edge.from_cell_id;
    int to_cell_ind = edge.to_cell_id;
    tare::MobilityType from_cell_type = tare::MobilityType::NONE;
    tare::MobilityType to_cell_type = tare::MobilityType::NONE;
    grid_world_ns::GridWorld::DecodeRoadmapEdgeWeight(edge.weight, from_cell_type, to_cell_type);
    grid_world->SetCellMobilityType(from_cell_ind, from_cell_type);
    grid_world->SetCellMobilityType(to_cell_ind, to_cell_type);
    geometry_msgs::Point from_cell_position = grid_world->GetCellPosition(from_cell_ind);
    geometry_msgs::Point to_cell_position = grid_world->GetCellPosition(to_cell_ind);
    grid_world->AddRoadmapNode(from_cell_ind, from_cell_position, from_cell_type);
    grid_world->AddRoadmapNode(to_cell_ind, to_cell_position, to_cell_type);
    grid_world->AddRoadmapTwoWayEdge(from_cell_ind, to_cell_ind);
  }
}

void MultiRobotExplorationManager::SyncGlobalKnowledge(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world)
{
  for (const auto& robot : robots_)
  {
    if (!robot.ExplorationInfoUpdated() || robot.GetID() == id_)
    {
      continue;
    }
    SyncCells(robot, grid_world);
    // SyncExploringCells(robot, grid_world);
    // SyncExploredCells(robot, grid_world);
    SyncRoadmap(robot, grid_world);
  }
}

void MultiRobotExplorationManager::SyncCells(const RobotManager& robot,
                                             const std::unique_ptr<grid_world_ns::GridWorld>& grid_world)
{
  std::vector<tare_msgs::Cell> cells;
  robot.GetCells(cells);
  for (const auto& cell : cells)
  {
    int cell_id = cell.id;
    int known_by_robot_ids = cell.known_by_robot_ids;
    int update_id = cell.update_id;
    int prev_update_id = grid_world->GetCellUpdateID(cell_id);
    grid_world_ns::CellStatus cell_status = static_cast<grid_world_ns::CellStatus>(cell.status);
    if (update_id >= prev_update_id)
    {
      grid_world->SyncCellWithIDs(cell_id, known_by_robot_ids, cell_status);
    }
  }
}

void MultiRobotExplorationManager::SyncRoadmap(const RobotManager& robot,
                                               const std::unique_ptr<grid_world_ns::GridWorld>& grid_world)
{
}

void MultiRobotExplorationManager::GetOutOfCommsRobotIndices(std::vector<int>& out_of_comms_robot_indices)
{
  out_of_comms_robot_indices.clear();
  out_of_comms_robot_indices.resize(robots_.size() + 1, false);
  MY_ASSERT(id_ >= 0 && id_ < out_of_comms_robot_indices.size());
  out_of_comms_robot_indices[id_] = true;
  for (const auto& robot : robots_)
  {
    if (robot.IsInComms())
    {
      int robot_id = robot.GetID();
      MY_ASSERT(robot_id >= 0 && robot_id < out_of_comms_robot_indices.size());

      out_of_comms_robot_indices[robot.GetID()] = true;
    }
  }
}

void MultiRobotExplorationManager::GetRobots(std::vector<Robot>& robots)
{
  int robot_num = robots_.size();
  for (int i = 0; i < robot_num; i++)
  {
    if (i == id_)
    {
      continue;
    }
    for (int j = 0; j < robot_num; j++)
    {
      if (j == i)
      {
        continue;
      }
      if (robots_[j].robot_update_secs_.empty() || robots_[j].robot_positions_.empty())
      {
        continue;
      }
      if (robots_[j].robot_update_secs_[i] > robots_[i].GetSecsFromStart())
      {
        robots_[i].SetInCommsPosition(robots_[j].robot_positions_[i]);
        robots_[i].SetSecsFromStart(robots_[j].robot_update_secs_[i]);
      }
    }
  }

  robots.clear();
  for (const auto& robot_manager : robots_)
  {
    robots.push_back(robot_manager.GetRobot());
  }
}

void MultiRobotExplorationManager::UpdateRobotPosition(const geometry_msgs::Point& position)
{
  robots_[id_].SetPosition(position);
  robots_[id_].SetInCommsPosition(position);
}

}  // namespace tare