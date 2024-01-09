/**
 * @file markov.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a Markov Decision Process (MDP) for the pursuit communication strategy
 * @version 0.1
 * @date 2023-05-11
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <iostream>
#include <vector>
#include <cmath>

enum MDPNodeType
{
  G,
  I,
  O
};

struct MDPNode
{
  MDPNodeType type_;
  int id_;
  MDPNode(MDPNodeType type, int id);
};

class PursuitMDP
{
private:
  double p_G_G_;
  double p_G_O_;
  double p_G_I_;
  double p_I_next_;
  double p_O_O_;
  double p_O_G_;
  double p_O_rest_;

public:
  PursuitMDP();
  void EvaluateMDP(const std::vector<double>& distances_between_nodes, double time_resolution, int time_steps,
                   std::vector<std::vector<double>>& G_values);
};