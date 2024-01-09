/**
 * @file multi_robot_exploration_manager.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that maintains the exploration information from other robots
 * @version 0.1
 * @date 2021-12-26
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PointStamped.h>

#include <utils/misc_utils.h>
#include <tare_msgs/ExplorationInfo.h>
#include <grid_world/grid_world.h>
#include <coordination/robot.h>

namespace tare
{
class RobotManager
{
public:
  explicit RobotManager(int id, tare::MobilityType mobility_type);
  ~RobotManager() = default;
  void InitializeSubscribers(ros::NodeHandle& nh);

  void ExplorationInfoCallback(const tare_msgs::ExplorationInfo::ConstPtr& exploration_info);
  void PositionCallback(const geometry_msgs::PointStamped::ConstPtr& position);
  void GetExplorationInfo(tare_msgs::ExplorationInfo& exploration_info) const
  {
    exploration_info = exploration_info_;
  }

  void GetCells(std::vector<tare_msgs::Cell>& cells) const;
  void GetExploringCellIDs(std::vector<int>& exploring_cell_ids) const;
  void GetRoadmapEdges(std::vector<std::pair<int, int>>& roadmap_edges) const;
  void GetRoadmapEdges(std::vector<tare_msgs::Edge>& roadmap_edges) const;
  bool ExplorationInfoUpdated() const
  {
    return exploration_info_updated_;
  }
  void SetExplorationInfoUpdated(bool updated)
  {
    exploration_info_updated_ = updated;
  }
  bool PositionUpdated() const
  {
    return position_updated_;
  }
  int GetID() const
  {
    return robot_.id_;
  }
  geometry_msgs::Point GetPosition() const
  {
    return robot_.position_;
  }
  geometry_msgs::Point GetInCommsPosition() const
  {
    return robot_.in_comms_position_;
  }
  void SetPosition(const geometry_msgs::Point& position)
  {
    robot_.position_ = position;
  }
  void SetInCommsPosition(const geometry_msgs::Point& in_comms_position)
  {
    robot_.in_comms_position_ = in_comms_position;
  }
  uint32_t GetSecsFromStart() const
  {
    return robot_.secs_from_start_;
  }
  void SetSecsFromStart(uint32_t secs_from_start)
  {
    robot_.secs_from_start_ = secs_from_start;
  }
  std::string GetName() const
  {
    return name_;
  }
  MobilityType GetType() const
  {
    return robot_.mobility_type_;
  }
  void SetInComms(bool in_comms)
  {
    robot_.in_comms_ = in_comms;
  }
  bool IsInComms() const
  {
    return robot_.in_comms_;
  }
  ros::Time GetInfoUpdateTime() const
  {
    return info_update_time_;
  }
  Robot GetRobot() const
  {
    return robot_;
  }
  bool IsRobotInRange(int robot_id)
  {
    return in_range_robot_ids_ & (1 << robot_id);
  }
  void ResetInRangeRobotIDs()
  {
    in_range_robot_ids_ = (1 << robot_.id_);
  }
  void SetRobotInRange(int robot_id)
  {
    in_range_robot_ids_ |= (1 << robot_id);
  }
  uint32_t GetInRangeRobotIDs()
  {
    return in_range_robot_ids_;
  }

  std::vector<geometry_msgs::Point> robot_positions_;
  std::vector<uint32_t> robot_update_secs_;

private:
  Robot robot_;
  std::string name_;
  tare_msgs::ExplorationInfo exploration_info_;
  bool exploration_info_updated_;
  bool position_updated_;
  ros::Time info_update_time_;
  int exploration_info_count_;
  uint32_t in_range_robot_ids_;

  ros::Subscriber exploration_info_sub_;
  ros::Subscriber position_sub_;
};

class MultiRobotExplorationManager
{
public:
  explicit MultiRobotExplorationManager(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~MultiRobotExplorationManager() = default;

  int GetSelfID()
  {
    return id_;
  }

  std::string GetSelfName()
  {
    return name_;
  }
  tare::MobilityType GetSelfType()
  {
    return type_;
  }
  void GetRoadmapEdges(std::vector<tare_msgs::Edge>& roadmap_edges) const;
  void UpdateCommsStatus(const geometry_msgs::Point& self_position);
  bool IsInComms();
  double GetCommsRange()
  {
    return kCommsRange;
  }
  void UpdateGlobalKnowledge(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world);
  void UpdateCells(const RobotManager& robot, const std::unique_ptr<grid_world_ns::GridWorld>& grid_world);
  void UpdateRoadmap(const RobotManager& robot, const std::unique_ptr<grid_world_ns::GridWorld>& grid_world);

  void SyncGlobalKnowledge(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world);
  void SyncCells(const RobotManager& robot, const std::unique_ptr<grid_world_ns::GridWorld>& grid_world);
  void SyncRoadmap(const RobotManager& robot, const std::unique_ptr<grid_world_ns::GridWorld>& grid_world);
  void GetOutOfCommsRobotIndices(std::vector<int>& out_of_comms_robot_indices);
  void GetRobots(std::vector<Robot>& robots);
  void UpdateRobotPosition(const geometry_msgs::Point& position);
  uint32_t GetInRangeRobotIDs()
  {
    return robots_[id_].GetInRangeRobotIDs();
  }

private:
  int id_;
  std::string name_;
  tare::MobilityType type_;
  geometry_msgs::Point position_;
  std::vector<std::string> robot_types_;
  std::vector<RobotManager> robots_;
  double kCommsRange;
  int kLostCommsSecThr;
  int kRobotNum;
  bool kHeterogeneous;
  std::string kTestID;
};
}  // namespace tare