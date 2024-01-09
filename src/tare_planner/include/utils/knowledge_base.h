/**
 * @file knowledgebase.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a knowledge base
 * @version 0.1
 * @date 2022-04-12
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

namespace tare
{
class KnowledgeBase
{
public:
  explicit KnowledgeBase();
  ~KnowledgeBase() = default;
  static void SetTotalNumber(int total_number);
  void SyncID(int id);
  void SyncIDs(int ids);
  bool IsIDSynced(int id);
  bool IsSynced();
  int GetSynced();
  int GetAllSynced();
  void Reset();

private:
  static int kAllSynced;
  int synced_;
};
}  // namespace tare
