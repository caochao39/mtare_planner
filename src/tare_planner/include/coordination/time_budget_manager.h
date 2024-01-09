/**
 * @file incremental_exploration_manager.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the incremental exploration behavior
 * @version 0.1
 * @date 2022-08-07
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
class TimeBudgetManager
{
public:
  explicit TimeBudgetManager();
  ~TimeBudgetManager() = default;
  int GetTimeBudget()
  {
    return time_budget_;
  }
  int GetRemainingExplorationTime()
  {
    int remaining_exploration_time = time_budget_ - (ros::Time::now() - exploration_start_time_).toSec();
    return remaining_exploration_time;
  }
  void SetTimeBudget(int time_budget)
  {
    time_budget_ = time_budget;
  }
  bool TimesUp()
  {
    int remaining_exploration_time = time_budget_ - (ros::Time::now() - exploration_start_time_).toSec();
    return remaining_exploration_time <= 0;
  }

private:
  ros::Time exploration_start_time_;
  int time_budget_;
};

}  // namespace tare