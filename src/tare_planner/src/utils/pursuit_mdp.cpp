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

#include <Eigen/Dense>
#include <utils/pursuit_mdp.h>

MDPNode::MDPNode(MDPNodeType node_type, int node_id) : type_(node_type), id_(node_id)
{
}

PursuitMDP::PursuitMDP()
  : p_G_G_(0.35), p_G_O_(0.15), p_G_I_(0.5), p_I_next_(1.0), p_O_O_(0.5), p_O_G_(0.3), p_O_rest_(0.5)
{
}

void PursuitMDP::EvaluateMDP(const std::vector<double>& distances_between_nodes, double time_resolution, int time_steps,
                             std::vector<std::vector<double>>& G_values)
{
  std::vector<MDPNode> nodes;
  for (int i = 0; i < distances_between_nodes.size(); i++)
  {
    // Add a G node
    MDPNode Gnode(MDPNodeType::G, i);
    nodes.push_back(Gnode);

    // Add I nodes
    int num_I_nodes = (int)(distances_between_nodes[i] / time_resolution) - 1;
    for (int j = 0; j < num_I_nodes; j++)
    {
      MDPNode Inode(MDPNodeType::I, i);
      nodes.push_back(Inode);
    }

    // Add an O node
    MDPNode Onode(MDPNodeType::O, i);
    nodes.push_back(Onode);
  }

  int num_nodes = nodes.size();
  Eigen::MatrixXd transition_matrix = Eigen::MatrixXd::Zero(num_nodes, num_nodes);

  int max_node_id = nodes.back().id_;

  for (int i = 0; i < num_nodes; i++)
  {
    for (int j = 0; j < num_nodes; j++)
    {
      MDPNode from_node = nodes[i];
      MDPNode to_node = nodes[j];

      if (from_node.type_ == MDPNodeType::O)
      {
        if (i < j)
        {
          int num_remaining_nodes = num_nodes - (i + 1) - (max_node_id - from_node.id_);
          if (to_node.type_ != MDPNodeType::O)
          {
            transition_matrix(i, j) = p_O_rest_ / num_remaining_nodes;
          }
        }
        else if (i == j)
        {
          transition_matrix(i, j) = p_O_O_;
          // The last O node
          if (i == num_nodes - 1)
          {
            transition_matrix(i, j) += p_O_rest_;
          }
        }
      }

      if (from_node.type_ == MDPNodeType::G)
      {
        if (j == i + 1)
        {
          transition_matrix(i, j) = p_G_I_;
        }
        else if (to_node.type_ == MDPNodeType::O && to_node.id_ == from_node.id_)
        {
          transition_matrix(i, j) = p_G_O_;
        }
        else if (i == j)
        {
          transition_matrix(i, j) = p_G_G_;
        }
      }

      if (from_node.type_ == MDPNodeType::I)
      {
        if ((j == i + 1 && to_node.type_ == MDPNodeType::I) || (j == i + 2 && to_node.type_ == MDPNodeType::G))
        {
          transition_matrix(i, j) = p_I_next_;
        }
      }
    }
  }

  std::vector<int> G_node_indices;
  for (int i = 0; i < nodes.size(); i++)
  {
    if (nodes[i].type_ == MDPNodeType::G)
    {
      G_node_indices.push_back(i);
    }
  }

  Eigen::VectorXd p_init = Eigen::VectorXd::Zero(num_nodes);
  p_init(0) = 1.0;

  Eigen::VectorXd p_final;

  G_values.clear();
  Eigen::MatrixXd Mt = Eigen::MatrixXd::Identity(num_nodes, num_nodes);
  for (int t = 0; t < time_steps; t++)
  {
    Mt *= transition_matrix;

    p_final = p_init.transpose() * Mt;

    // Get values corresponding to G nodes
    std::vector<double> G_value_t;
    for (int i = 0; i < G_node_indices.size(); i++)
    {
      G_value_t.push_back(p_final(G_node_indices[i]));
    }
    G_values.push_back(G_value_t);
  }
}
