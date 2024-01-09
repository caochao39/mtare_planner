/**
 * @file knowledge_base.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a knowledge base
 * @version 0.1
 * @date 2022-04-12
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <utils/knowledge_base.h>

namespace tare
{
int KnowledgeBase::kAllSynced = 0;

KnowledgeBase::KnowledgeBase() : synced_(0)
{
}

void KnowledgeBase::SetTotalNumber(int total_number)
{
  kAllSynced = (1 << total_number) - 1;
}

void KnowledgeBase::SyncID(int id)
{
  synced_ |= 1 << id;
}

void KnowledgeBase::SyncIDs(int ids)
{
  synced_ |= ids;
}

bool KnowledgeBase::IsIDSynced(int id)
{
  return synced_ & (1 << id);
}

bool KnowledgeBase::IsSynced()
{
  return synced_ >= kAllSynced;
}

int KnowledgeBase::GetSynced()
{
  return synced_;
}

int KnowledgeBase::GetAllSynced()
{
  return kAllSynced;
}
void KnowledgeBase::Reset()
{
  synced_ = 0;
}

}  // namespace tare