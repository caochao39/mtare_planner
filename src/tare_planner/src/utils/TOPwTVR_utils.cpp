/**
 * @file TOPwTVR_utils.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Solve a Team Orienteering Problem with Time-Varying Reward problem
 * @version 0.1
 * @date 2023-05-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <iostream>
#include <functional>
#include <climits>
#include <algorithm>
#include <utils/TOPwTVR.h>
#include <utils/TOPwTVR_utils.h>

IndexSet::IndexSet()
{
  element_num_ = 300;
  unsigned long num_of_bit_sets = element_num_ / BIT_SET_SIZE + 1;
  bits_.resize(num_of_bit_sets, 0);
}

IndexSet::IndexSet(unsigned long element_num) : element_num_(element_num)
{
  unsigned long num_of_bit_sets = element_num_ / BIT_SET_SIZE + 1;
  bits_.resize(num_of_bit_sets, 0);
}

void IndexSet::IncludeAllElements()
{
  for (int i = 0; i < bits_.size(); i++)
  {
    bits_[i] = ~0UL;
  }
}

void IndexSet::SetElementNum(unsigned long element_num)
{
  element_num_ = element_num;
  unsigned long num_of_bit_sets = element_num_ / BIT_SET_SIZE + 1;
  bits_.resize(num_of_bit_sets, 0);
}

bool IndexSet::IsElementInSet(unsigned long index) const
{
  unsigned long bit_set_index = index / BIT_SET_SIZE;
  return bits_[bit_set_index] & (1 << (index - bit_set_index * BIT_SET_SIZE));
}

bool IndexSet::IsSubset(const IndexSet& other) const
{
  unsigned long num_bit_set = bits_.size();
  for (int i = 0; i < num_bit_set; i++)
  {
    if ((bits_[i] & other.bits_[i]) != bits_[i])
    {
      return false;
    }
  }
  return true;
}

void IndexSet::Insert(unsigned long index)
{
  unsigned long bit_set_index = index / BIT_SET_SIZE;
  bits_[bit_set_index] |= (1 << (index - bit_set_index * BIT_SET_SIZE));
}

void IndexSet::Remove(unsigned long index)
{
  unsigned long bit_set_index = index / BIT_SET_SIZE;
  bits_[bit_set_index] &= ~(1 << (index - bit_set_index * BIT_SET_SIZE));
}

void IndexSet::Print()
{
  for (int i = 0; i < element_num_; i++)
  {
    if (IsElementInSet(i))
    {
      std::cout << i << " ";
    }
  }
  std::cout << std::endl;
}

std::ostream& operator<<(std::ostream& os, const IndexSet& set)
{
  std::string bits_str;
  int bit_count = 0;
  for (int i = 0; i < set.bits_.size(); i++)
  {
    for (int j = 0; j < set.BIT_SET_SIZE; j++)
    {
      if (set.bits_[i] & (1UL << j))
      {
        bits_str += std::to_string(bit_count) + "(1) ";
      }
      else
      {
        bits_str += std::to_string(bit_count) + "(0) ";
      }
      bit_count++;
    }
  }
  os << bits_str;
  return os;
}

double EvaluatePathProfit(std::vector<int>& path, const std::vector<std::vector<int>>& distance_matrix,
                          const std::vector<std::vector<double>>& profit, bool accumulate_reward_at_same_node)
{
  std::vector<int> time_stamps;
  int accumulated_t = -1;
  double accumulated_profit = 0;
  for (int i = 0; i < path.size(); i++)
  {
    int delta_t = 1;
    double delta_profit = 0.0;
    if (i > 0)
    {
      int curr_node_id = path[i];
      int prev_node_id = path[i - 1];
      if (curr_node_id != prev_node_id)
      {
        delta_t = distance_matrix[prev_node_id][curr_node_id];
        delta_profit = profit[path[i]][accumulated_t + delta_t];
      }
    }

    accumulated_t += delta_t;
    if (accumulate_reward_at_same_node)
    {
      accumulated_profit += profit[path[i]][accumulated_t];
    }
    else
    {
      accumulated_profit += delta_profit;
    }
    time_stamps.push_back(accumulated_t);
    // std::cout << path[i] << "(" << accumulated_t << ":" << profit[path[i]][accumulated_t] << ") ";
  }
  // std::cout << std::endl;

  // std::cout << "accumulated profit: " << accumulated_profit << std::endl;

  return accumulated_profit;
}

double EvaluateMultiVehiclePathProfit(std::vector<std::vector<int>>& paths,
                                      const std::vector<std::vector<int>>& distance_matrix,
                                      const std::vector<std::vector<double>>& profit,
                                      bool accumulate_reward_at_same_node)
{
  double accumulated_profit = 0;

  int num_vehicle = paths.size();

  for (int vehicle_id = 0; vehicle_id < num_vehicle; vehicle_id++)
  {
    std::vector<int> path = paths[vehicle_id];

    std::vector<int> time_stamps;
    int accumulated_t = -1;
    for (int i = 0; i < path.size(); i++)
    {
      int delta_t = 1;
      double delta_profit = 0.0;
      if (i > 0)
      {
        int curr_node_id = path[i];
        int prev_node_id = path[i - 1];
        if (curr_node_id != prev_node_id)
        {
          delta_t = distance_matrix[prev_node_id][curr_node_id];
          delta_profit = profit[path[i]][accumulated_t + delta_t];
        }
      }

      accumulated_t += delta_t;
      if (accumulate_reward_at_same_node)
      {
        accumulated_profit += profit[path[i]][accumulated_t];
      }
      else
      {
        accumulated_profit += delta_profit;
      }
      time_stamps.push_back(accumulated_t);
    }
  }

  return accumulated_profit;
}
