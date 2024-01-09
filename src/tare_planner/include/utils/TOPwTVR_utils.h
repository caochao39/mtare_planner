/**
 * @file TOPwTVR_utils.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that solves a Team Orienteering Problem with Time-Varying Reward problem
 * @version 0.1
 * @date 2023-05-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <nav_msgs/Path.h>

class IndexSet
{
public:
  IndexSet();

  IndexSet(unsigned long element_num);

  void IncludeAllElements();

  void SetElementNum(unsigned long element_num);

  bool IsElementInSet(unsigned long index) const;

  bool IsSubset(const IndexSet& other) const;

  void Insert(unsigned long index);

  void Remove(unsigned long index);

  void Print();

  bool operator==(const IndexSet& other) const
  {
    for (int i = 0; i < bits_.size(); i++)
    {
      if (bits_[i] != other.bits_[i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const IndexSet& other) const
  {
    for (int i = 0; i < bits_.size(); i++)
    {
      if (bits_[i] != other.bits_[i])
      {
        return true;
      }
    }
    return false;
  }
  friend std::ostream& operator<<(std::ostream& os, const IndexSet& set);

private:
  // std::bitset<256> bits;  // Assumes a maximum of 256 indices
  std::vector<unsigned long> bits_;
  static const unsigned long BIT_SET_SIZE = 32;
  unsigned long element_num_;
};

double EvaluatePathProfit(std::vector<int>& path, const std::vector<std::vector<int>>& distance_matrix,
                          const std::vector<std::vector<double>>& profit, bool accumulate_reward_at_same_node);
double EvaluateMultiVehiclePathProfit(std::vector<std::vector<int>>& paths,
                                      const std::vector<std::vector<int>>& distance_matrix,
                                      const std::vector<std::vector<double>>& profit,
                                      bool accumulate_reward_at_same_node);

// int GetRemainingNodeSet(std::vector<int>& path);