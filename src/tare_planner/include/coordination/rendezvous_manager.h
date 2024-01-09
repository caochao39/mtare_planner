/**
 * @file rendezvous_manager.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the rendezvous schedule
 * @version 0.1
 * @date 2022-07-30
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
#include <coordination/robot.h>

namespace tare
{
class RendezvousManager
{
public:
  explicit RendezvousManager();
  ~RendezvousManager() = default;
  // geometry_msgs::Point GetRendezvousPosition()
  // {
  //   return rendezvous_position_;
  // }
  int GetCurrentRendezvousCellID()
  {
    return curr_rendezvous_cell_id_;
  }
  int GetNextRendezvousCellID()
  {
    return next_rendezvous_cell_id_;
  }
  void SetCurrentRendezvousCellID(int cell_id)
  {
    curr_rendezvous_cell_id_ = cell_id;
    initial_rendezvous_set_ = true;
  }
  void SetNextrRendezvousCellID(int cell_id)
  {
    next_rendezvous_cell_id_ = cell_id;
  }
  int GetCurrentRendezvousTimeInterval()
  {
    return curr_rendezvous_time_interval_;
  }

  int GetNextRendezvousTimeInterval()
  {
    return next_rendezvous_time_interval_;
  }
  void SetCurrentRendezvousTimeInterval(int time_interval)
  {
    curr_rendezvous_time_interval_ = time_interval;
  }
  void SetNextRendezvousTimeInterval(int time_interval)
  {
    next_rendezvous_time_interval_ = time_interval;
    ;
  }
  void PlanForNextRendezvous(const std::vector<tare::Robot>& robots, int self_id,
                             const std::vector<int>& global_exploring_cell_ids,
                             const std::vector<std::vector<std::vector<int>>>& distance_matrices,
                             int cell_to_cell_distance_offset);
  void ResetPlannedNextRendezvous()
  {
    planned_next_rendezvous_ = false;
  }
  void SyncNextRendezvous(const std::vector<tare::Robot>& robots, int self_id);
  bool TimeToMeet(int time_to_rendezvous);
  int GetRemainingTimeToMeet();
  bool InitialRendezvousSet()
  {
    return initial_rendezvous_set_;
  }
  void ResetTimer()
  {
    exploration_start_time_ = ros::Time::now();
  }
  void UpdateCurrentRendezvousPlan()
  {
    curr_rendezvous_cell_id_ = next_rendezvous_cell_id_;
    curr_rendezvous_time_interval_ = next_rendezvous_time_interval_;
  }

private:
  int next_rendezvous_cell_id_;
  int next_rendezvous_time_interval_;

  int curr_rendezvous_cell_id_;
  int curr_rendezvous_time_interval_;

  int min_rendezvous_time_interval_;
  int max_rendezvous_time_interval_;
  ros::Time exploration_start_time_;
  bool initial_rendezvous_set_;
  bool planned_next_rendezvous_;
};

}  // namespace tare