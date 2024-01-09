/**
 * @file robot.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that maintains the information of a robot
 * @version 0.1
 * @date 2022-07-13
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

namespace tare
{
enum class MobilityType
{
  NONE = 0,
  WHEELED = 1,
  TRACKED = 2,
  AERIAL = 3
};
bool Traversable(tare::MobilityType robot_type, tare::MobilityType terrain_type);
MobilityType StringToMobilityType(std::string type_name);
std::string MobilityTypeToString(MobilityType type);

struct VRPCost
{
  int dropped_nodes_number_;
  int longest_route_length_;
  VRPCost();
  VRPCost(int dropped_nodes_number, int longest_route_length);
  friend std::ostream& operator<<(std::ostream& out, const VRPCost& cost);
};
struct VRPPlan
{
  VRPCost cost_;
  int32_t sampled_robots_;
  std::vector<int> local_exploring_cell_ids_;
  std::vector<std::vector<int>> no_comms_ordered_cell_ids_;
  std::vector<std::vector<int>> relay_comms_ordered_cell_ids_;
  std::vector<std::vector<int>> assume_comms_ordered_cell_ids_;
};

void DecodeGlobalVRPPlan(const std::vector<int32_t>& msg, VRPPlan& vrp_plan);
void DecodeToOrderedCellIDs(const std::vector<int>& msg, std::vector<std::vector<int>>& ordered_cell_ids);
bool LessCost(const tare::VRPCost& cost1, const VRPCost& cost2);

enum class CommsStatus
{
  IN_COMMS = 0,
  RELAY_COMMS = 1,
  CONVOY = 2,
  LOST_COMMS = 3

};

struct Robot
{
  int id_;
  uint32_t secs_from_start_;
  bool in_comms_;
  bool lost_;
  bool finished_exploration_;
  bool visited_for_convoy_;
  tare::MobilityType mobility_type_;
  geometry_msgs::Point position_;
  geometry_msgs::Point in_comms_position_;
  explicit Robot(int id, tare::MobilityType mobility_type);
  ~Robot() = default;
  VRPPlan vrp_plan_;
  int rendezvous_cell_id_;
};
}  // namespace tare