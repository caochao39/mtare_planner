/**
 * @file incremental_exploration_manager.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the incremental exploration behavior
 * @version 0.1
 * @date 2022-08-07
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <coordination/time_budget_manager.h>

namespace tare
{
TimeBudgetManager::TimeBudgetManager()
{
  exploration_start_time_ = ros::Time::now();
}

}  // namespace tare