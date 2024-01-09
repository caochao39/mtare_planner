/**
 * @file robot.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that maintains the information of a robot
 * @version 0.1
 * @date 2022-07-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <coordination/robot.h>

namespace tare
{
Robot::Robot(int id, tare::MobilityType mobility_type)
  : id_(id)
  , secs_from_start_(0)
  , in_comms_(true)
  , lost_(false)
  , finished_exploration_(true)
  , visited_for_convoy_(false)
  , mobility_type_(mobility_type)
{
  position_.x = 0;
  position_.y = 0;
  position_.z = 0;

  in_comms_position_ = position_;
}

bool Traversable(tare::MobilityType robot_type, tare::MobilityType terrain_type)
{
  return robot_type == terrain_type || terrain_type == tare::MobilityType::NONE;
}

MobilityType StringToMobilityType(std::string type_name)
{
  if (type_name == "wheeled")
  {
    return MobilityType::WHEELED;
  }
  else if (type_name == "tracked")
  {
    return MobilityType::TRACKED;
  }
  else if (type_name == "aerial")
  {
    return MobilityType::AERIAL;
  }
  else
  {
    ROS_INFO_STREAM("Invalid robot type: " << type_name << ", setting to wheeled by default");
    return MobilityType::WHEELED;
  }
}

std::string MobilityTypeToString(MobilityType type)
{
  switch (type)
  {
    case MobilityType::WHEELED: {
      return "wheeled";
    }
    case MobilityType::TRACKED: {
      return "tracked";
    }
    case MobilityType::AERIAL: {
      return "aerial";
    }
    default: {
      ROS_ERROR_STREAM("Wrong robot type");
      return "wheeled";
    }
  }
}

VRPCost::VRPCost() : dropped_nodes_number_(INT_MAX), longest_route_length_(misc_utils_ns::INF_DISTANCE)
{
}
VRPCost::VRPCost(int dropped_nodes_number, int longest_route_length)
  : dropped_nodes_number_(dropped_nodes_number), longest_route_length_(longest_route_length)
{
}

std::ostream& operator<<(std::ostream& out, const VRPCost& cost)
{
  out << "dropped: " << cost.dropped_nodes_number_ << " route: " << cost.longest_route_length_;
  return out;
}

bool LessCost(const VRPCost& cost1, const VRPCost& cost2)
{
  if (cost1.dropped_nodes_number_ < cost2.dropped_nodes_number_)
  {
    return true;
  }
  else if (cost1.dropped_nodes_number_ > cost2.dropped_nodes_number_)
  {
    return false;
  }
  else
  {
    if (cost1.longest_route_length_ < cost2.longest_route_length_)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

void DecodeGlobalVRPPlan(const std::vector<int32_t>& msg, VRPPlan& vrp_plan)
{
  int msg_length = msg.size();
  if (msg_length == 0)
  {
    vrp_plan.cost_.dropped_nodes_number_ = INT_MAX;
    vrp_plan.cost_.longest_route_length_ = misc_utils_ns::INF_DISTANCE;
    vrp_plan.sampled_robots_ = 0;
    vrp_plan.no_comms_ordered_cell_ids_.clear();
    vrp_plan.relay_comms_ordered_cell_ids_.clear();
    vrp_plan.assume_comms_ordered_cell_ids_.clear();
    return;
  }

  vrp_plan.sampled_robots_ = msg[0];

  vrp_plan.no_comms_ordered_cell_ids_.clear();
  vrp_plan.relay_comms_ordered_cell_ids_.clear();
  vrp_plan.assume_comms_ordered_cell_ids_.clear();

  std::vector<int> no_comms_plan_msg;
  std::vector<int> relay_comms_plan_msg;
  std::vector<int> assume_comms_plan_msg;
  int break_count = 0;
  for (int i = 1; i < msg_length; i++)
  {
    if (msg[i] == -2)
    {
      break_count++;
    }
    else
    {
      if (break_count == 0)
      {
        no_comms_plan_msg.push_back(msg[i]);
      }
      else if (break_count == 1)
      {
        relay_comms_plan_msg.push_back(msg[i]);
      }
      else if (break_count == 2)
      {
        assume_comms_plan_msg.push_back(msg[i]);
      }
      else
      {
        ROS_ERROR_STREAM("Error in decoding vrp plan: break count " << break_count);
      }
    }
  }

  DecodeToOrderedCellIDs(no_comms_plan_msg, vrp_plan.no_comms_ordered_cell_ids_);
  DecodeToOrderedCellIDs(relay_comms_plan_msg, vrp_plan.relay_comms_ordered_cell_ids_);
  DecodeToOrderedCellIDs(assume_comms_plan_msg, vrp_plan.assume_comms_ordered_cell_ids_);
}

void DecodeToOrderedCellIDs(const std::vector<int>& msg, std::vector<std::vector<int>>& ordered_cell_ids)
{
  ordered_cell_ids.clear();
  std::vector<int> route;
  for (int i = 0; i < msg.size(); i++)
  {
    if (msg[i] == -1)
    {
      ordered_cell_ids.push_back(route);
      route.clear();
    }
    else
    {
      route.push_back(msg[i]);
    }
  }
}
}  // namespace tare