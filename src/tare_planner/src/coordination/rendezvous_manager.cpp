/**
 * @file rendezvous_manager.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the rendezvous schedule
 * @version 0.1
 * @date 2022-07-31
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <coordination/rendezvous_manager.h>

namespace tare
{
RendezvousManager::RendezvousManager()
  : curr_rendezvous_cell_id_(-1)
  , next_rendezvous_cell_id_(-1)
  , curr_rendezvous_time_interval_(120)
  , next_rendezvous_time_interval_(120)
  , min_rendezvous_time_interval_(120)
  , max_rendezvous_time_interval_(300)
  , initial_rendezvous_set_(false)
  , planned_next_rendezvous_(false)
{
  exploration_start_time_ = ros::Time::now();
}

void RendezvousManager::PlanForNextRendezvous(const std::vector<tare::Robot>& robots, int self_id,
                                              const std::vector<int>& global_exploring_cell_ids,
                                              const std::vector<std::vector<std::vector<int>>>& distance_matrices,
                                              int cell_to_cell_distance_offset)
{
  exploration_start_time_ = ros::Time::now();
  bool got_rendezvous_position_from_peer_robots = false;
  for (int i = 0; i < robots.size(); i++)
  {
    if (i == self_id)
    {
      continue;
    }
    // if (robots[i].in_comms_ && i < self_id)
    if (robots[i].in_comms_ && i == 0)  // tmp fix: only get from robot 0
    {
      if (!robots[i].vrp_plan_.relay_comms_ordered_cell_ids_.empty() &&
          !robots[i].vrp_plan_.relay_comms_ordered_cell_ids_.front().size() < 2)
      {
        next_rendezvous_cell_id_ = robots[i].vrp_plan_.relay_comms_ordered_cell_ids_.front().front();
        next_rendezvous_time_interval_ = robots[i].vrp_plan_.relay_comms_ordered_cell_ids_.front()[1];
        got_rendezvous_position_from_peer_robots = true;
        break;
      }
    }
  }
  if (got_rendezvous_position_from_peer_robots || planned_next_rendezvous_)
  {
    return;
  }

  if (global_exploring_cell_ids.size() == 1)
  {
    next_rendezvous_cell_id_ = global_exploring_cell_ids[0];
    next_rendezvous_time_interval_ = min_rendezvous_time_interval_;
    return;
  }

  int robot_num = distance_matrices.size();
  std::vector<std::vector<int>> condense_matrix(global_exploring_cell_ids.size(),
                                                std::vector<int>(global_exploring_cell_ids.size(), INT_MAX));
  for (int i = 0; i < condense_matrix.size(); i++)
  {
    for (int j = 0; j < condense_matrix[i].size(); j++)
    {
      for (int robot_id = 0; robot_id < robot_num; robot_id++)
      {
        int dm_i = i + robot_num;
        int dm_j = j + robot_num;
        condense_matrix[i][j] = std::min(condense_matrix[i][j], distance_matrices[robot_id][dm_i][dm_j]);
      }
    }
  }

  std::vector<int> max_lengths(global_exploring_cell_ids.size(), INT_MIN);
  for (int i = 0; i < max_lengths.size(); i++)
  {
    int max_element_index = 0;
    max_lengths[i] =
        misc_utils_ns::GetMaxElement<int>(condense_matrix[i], max_element_index) - cell_to_cell_distance_offset;
  }

  int min_length_index = 0;
  int min_length = misc_utils_ns::GetMinElement<int>(max_lengths, min_length_index);

  MY_ASSERT(min_length_index >= 0 && min_length_index < global_exploring_cell_ids.size());

  int distance_to_farthest_cell = max_lengths[min_length_index];

  MY_ASSERT(distance_to_farthest_cell < misc_utils_ns::INF_DISTANCE);

  ROS_INFO_STREAM("cell max lengths: ");
  misc_utils_ns::PrintVector<int>(max_lengths);

  ROS_INFO_STREAM(misc_utils_ns::ColoredText("Distance to farthest cell: " + std::to_string(distance_to_farthest_cell),
                                             misc_utils_ns::TextColor::MAGENTA));

  next_rendezvous_time_interval_ = min_rendezvous_time_interval_ + distance_to_farthest_cell / 2;
  next_rendezvous_time_interval_ = std::min(next_rendezvous_time_interval_, max_rendezvous_time_interval_);

  next_rendezvous_cell_id_ = global_exploring_cell_ids[min_length_index];

  planned_next_rendezvous_ = true;
}

void RendezvousManager::SyncNextRendezvous(const std::vector<tare::Robot>& robots, int self_id)
{
  for (int i = 0; i < robots.size(); i++)
  {
    if (i == self_id)
    {
      continue;
    }
    if (robots[i].in_comms_ && i < self_id)
    {
      if (!robots[i].vrp_plan_.relay_comms_ordered_cell_ids_.empty() &&
          !robots[i].vrp_plan_.relay_comms_ordered_cell_ids_.front().size() < 2)
      {
        next_rendezvous_cell_id_ = robots[i].vrp_plan_.relay_comms_ordered_cell_ids_.front()[0];
        next_rendezvous_time_interval_ = robots[i].vrp_plan_.relay_comms_ordered_cell_ids_.front()[1];
        break;
      }
    }
  }
}

bool RendezvousManager::TimeToMeet(int time_to_rendezvous)
{
  int remaining_exploration_time =
      curr_rendezvous_time_interval_ - (ros::Time::now() - exploration_start_time_).toSec();
  ROS_INFO_STREAM("exploration time since start: " << (ros::Time::now() - exploration_start_time_).toSec());
  ROS_INFO_STREAM("remaining time: " << remaining_exploration_time);
  // if (time_to_rendezvous >= remaining_exploration_time)
  if (remaining_exploration_time < 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int RendezvousManager::GetRemainingTimeToMeet()
{
  int remaining_exploration_time =
      curr_rendezvous_time_interval_ - (ros::Time::now() - exploration_start_time_).toSec();
  return remaining_exploration_time;
}

}  // namespace tare