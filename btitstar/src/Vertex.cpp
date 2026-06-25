/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, University of Oxford
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the names of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Authors: Yi Wang
#include "ompl/geometric/planners/lazyinformedtrees/btitstar/Vertex.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <string>

#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalStates.h"

using namespace std::string_literals;

namespace ompl
{
    namespace geometric
    {
        namespace btitstar
        {
            namespace
            {
                std::size_t generateId()
                {
                    static std::atomic<std::size_t> id{0u};
                    return id++;
                }
            }  // namespace

            Vertex::Vertex(const ompl::base::SpaceInformationPtr &spaceInformation,
                           const ompl::base::ProblemDefinitionPtr &problemDefinition, const std::size_t &batchId)
              : spaceInformation_(spaceInformation)
              , problemDefinition_(problemDefinition)
              , objective_(problemDefinition->getOptimizationObjective())
              , forwardChildren_()
              , reverseChildren_()
              , forwardParent_()
              , reverseParent_()
              , state_(spaceInformation->allocState())  // The memory allocated here is freed in the destructor.
              , costToComeFromStart_(objective_->infiniteCost())
              , edgeCostFromForwardParent_(objective_->infiniteCost())
              , costToComeFromGoal_(objective_->infiniteCost())
              , expandedCostToComeFromGoal_(objective_->infiniteCost())
              , costToGoToGoal_(objective_->infiniteCost())
              , vertexId_(generateId())
              , batchId_(batchId)
            {
            }

            Vertex::Vertex(const std::shared_ptr<Vertex> &other)
              : spaceInformation_(other->spaceInformation_)
              , problemDefinition_(other->problemDefinition_)
              , objective_(other->objective_)
              , forwardChildren_(other->forwardChildren_)
              , reverseChildren_(other->reverseChildren_)
              , forwardParent_(other->forwardParent_)
              , reverseParent_(other->reverseParent_)
              , state_(spaceInformation_->allocState())  // The memory allocated here is freed in the destructor.
              , costToComeFromStart_(other->costToComeFromStart_)
              , edgeCostFromForwardParent_(other->edgeCostFromForwardParent_)
              , costToComeFromGoal_(other->costToComeFromGoal_)
              , expandedCostToComeFromGoal_(other->expandedCostToComeFromGoal_)
              , costToGoToGoal_(other->costToGoToGoal_)
              , vertexId_(other->vertexId_)
              , batchId_(other->batchId_)
            {
                spaceInformation_->copyState(state_, other->getState());
            }

            Vertex::~Vertex()
            {
                // The state has associated memory that needs to be freed manually.
                spaceInformation_->freeState(state_);
            };

            std::size_t Vertex::getId() const
            {
                return vertexId_;
            }

            ompl::base::State *Vertex::getState()
            {
                return state_;
            }

            ompl::base::State const *Vertex::getState() const
            {
                return state_;
            }

            ompl::base::ScopedState<> Vertex::getScopedState() const
            {
                return ompl::base::ScopedState<>(spaceInformation_->getStateSpace(), state_);
            }

            ompl::base::Cost Vertex::getCostToComeFromStart() const
            {
                return costToComeFromStart_;
            }

            ompl::base::Cost Vertex::getCostToComeFromGoal() const
            {
                return costToComeFromGoal_;
            }


            double Vertex::getForwardEdgeTime()
            {
                 return forwardEdgeTime_;
            }

            double Vertex::getReverseEdgeTime()
            {
                 return reverseEdgeTime_;
            }

            ompl::base::Cost Vertex::getCostToGoToGoal() const
            {
                return getCostToComeFromGoal();
            }

            void Vertex::setStart()
            {
                 isStartState_ = true;  
            }
                
            bool Vertex::isStart()
            {
                  return isStartState_;
            }
                
            void Vertex::setGoal()
            {
                  isGoalState_ = true; 
            }
                
            bool Vertex::isGoal()
            {
                  return isGoalState_;  
            }
             
            bool Vertex::pivotalState()
            {
                   return pivotalState_;
            }

            void Vertex::setPivotalState()
            {
                   pivotalState_ = true; 
            }
                
            void Vertex::resetPivotalState()
            { 
                   pivotalState_ = false;  
            }
                
            bool Vertex::postSoluState()
            {
                 return postSolState_; 
            }
                
            void Vertex::setPostSoluState()
            {
                  postSolState_ = true; 
            }
                
            void Vertex::resetPostSoluState()
            {
                   postSolState_ = false;  
            }

            bool Vertex::hasForwardParent() const
            {
                return static_cast<bool>(forwardParent_.lock());
            }

            std::shared_ptr<Vertex> Vertex::getForwardParent() const
            {
                return forwardParent_.lock();
            }

            bool Vertex::hasReverseParent() const
            {
                return static_cast<bool>(reverseParent_.lock());
            }

            std::shared_ptr<Vertex> Vertex::getReverseParent() const
            {
                return reverseParent_.lock();
            }

            ompl::base::Cost Vertex::getForwardEdgeCost()
            {
                return edgeCostFromForwardParent_;
            }

            ompl::base::Cost Vertex::getReverseEdgeCost()
            {
                return edgeCostFromReverseParent_;
            }

            void Vertex::setCostToComeFromStart(const ompl::base::Cost &cost)
            {
                costToComeFromStart_ = cost;
            }

            void Vertex::setCostToComeFromGoal(const ompl::base::Cost &cost)
            {
                costToComeFromGoal_ = cost;
            }

            void Vertex::setLowerCostBoundToStart(const ompl::base::Cost &ToStart)
            {
                  lowerCostBoundToGoToStart_ = ToStart;      
            }             

            void Vertex::setLowerCostBoundToGoal(const ompl::base::Cost &ToGoal)
            {
                  lowerCostBoundToGoToGoal_ = ToGoal;  
            }   
              
            ompl::base::Cost Vertex::getLowerCostBoundToStart()
            {
                  return lowerCostBoundToGoToStart_;      
            }             

            ompl::base::Cost Vertex::getLowerCostBoundToGoal()
            {
                  return lowerCostBoundToGoToGoal_;  
            } 

            void Vertex::resetCostToComeFromGoal()
            {
                costToComeFromGoal_ = objective_->infiniteCost();
            }

            void Vertex::setCostToGoToGoal(const ompl::base::Cost &cost)
            {
                costToGoToGoal_ = cost;
            }
            
            void Vertex::setEdgeTime(const double &time_)
            {
                    EdgeTime_ = time_;  
            }
            
            double Vertex::getEdgeTime()
            {
                    return EdgeTime_;
            }
            
            void Vertex::resetEdgeTime()
            {
                    EdgeTime_ = 0;
            }
            
            bool Vertex::meetingState()
            {
                   return isMeetState_;
            }
            
            void Vertex::setMeetingState()
            {
                    isMeetState_ = true;
            }
            
            void Vertex::resetMeetingState()
            {
                    isMeetState_ = false;
            }

            void Vertex::setForwardParent(const std::shared_ptr<Vertex> &vertex, const ompl::base::Cost &edgeCost, double edgeTime)
            {
                // If this is a rewiring, remove from my parent's children.
                if (static_cast<bool>(forwardParent_.lock()))
                {
                    forwardParent_.lock()->removeFromForwardChildren(vertexId_);
                }

                // Remember the edge cost.
                edgeCostFromForwardParent_ = edgeCost;
                
                forwardEdgeTime_ = edgeTime;

                // Remember the corresponding parent.
                forwardParent_ = std::weak_ptr<Vertex>(vertex);

                // Update the cost to come.
                costToComeFromStart_ = objective_->combineCosts(vertex->getCostToComeFromStart(), edgeCost);
            }

            void Vertex::resetForwardParent()
            {
                forwardParent_.reset();
            }

            void Vertex::setReverseParent(const std::shared_ptr<Vertex> &vertex, const ompl::base::Cost &edgeCost, double edgeTime)
            {
                // If this is a rewiring, remove from my parent's children.
                if (static_cast<bool>(reverseParent_.lock()))
                {
                    reverseParent_.lock()->removeFromReverseChildren(vertexId_);
                }
                edgeCostFromReverseParent_ = edgeCost;
                
                reverseEdgeTime_ = edgeTime;
                
                // Remember the parent.
                reverseParent_ = std::weak_ptr<Vertex>(vertex);
                
                costToComeFromGoal_ = objective_->combineCosts(vertex->getCostToComeFromGoal(), edgeCost);
            }
            
            bool Vertex::forwardExpanded()
            {
                   return isForwardExpanded_;             
            }
            
            bool Vertex::reverseExpanded()
            {
                   return isReverseExpanded_;
            }
                
            void Vertex::setForwardExpanded()
            {
                    isForwardExpanded_ = true;
            }
            
            void Vertex::setReverseExpanded()
            {
                    isReverseExpanded_ = true;
            }
                
            void Vertex::resetForwardExpanded()
            {
                    isForwardExpanded_ = false;
            }
            void Vertex::resetReverseExpanded()
            {
                    isReverseExpanded_ = false; 
            }
            
            void Vertex::resetReverseParent()
            {
                reverseParent_.reset();
            }

            void Vertex::addToForwardChildren(const std::shared_ptr<Vertex> &vertex)
            {
                forwardChildren_.emplace_back(vertex);
            }

            void Vertex::removeFromForwardChildren(std::size_t vertexId)
            {
                // Find the child.
                auto it = std::find_if(
                    forwardChildren_.begin(), forwardChildren_.end(),
                    [vertexId](const std::weak_ptr<Vertex> &child) { return vertexId == child.lock()->getId(); });

                // Throw if it is not found.
                if (it == forwardChildren_.end())
                {
                    auto msg = "Asked to remove vertex from forward children that is currently not a child."s;
                    throw ompl::Exception(msg);
                }

                // Swap and pop.
                std::iter_swap(it, forwardChildren_.rbegin());
                forwardChildren_.pop_back();
            }

            void Vertex::addToReverseChildren(const std::shared_ptr<Vertex> &vertex)
            {
                reverseChildren_.push_back(vertex);
            }

            void Vertex::removeFromReverseChildren(std::size_t vertexId)
            {
                // Find the child.
                auto it = std::find_if(
                    reverseChildren_.begin(), reverseChildren_.end(),
                    [vertexId](const std::weak_ptr<Vertex> &child) { return vertexId == child.lock()->getId(); });

                // Throw if it is not found.
                if (it == reverseChildren_.end())
                {
                    auto msg = "Asked to remove vertex from reverse children that is currently not a child."s;
                    throw ompl::Exception(msg);
                }

                // Swap and pop.
                std::iter_swap(it, reverseChildren_.rbegin());
                reverseChildren_.pop_back();
            }

            void Vertex::whitelistAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                whitelistedChildren_.emplace_back(vertex);
            }

            bool Vertex::isWhitelistedAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                // Check if the vertex is whitelisted by iterating over all whitelisted children.
                // It this detects an invalid vertex, e.g., a vertex that was once whitelisted but
                // has been pruned since, remove the vertex from the list of whitelisted children.
                auto it = whitelistedChildren_.begin();
                while (it != whitelistedChildren_.end())
                {
                    // Check if the child is a valid vertex.
                    if (const auto child = it->lock())
                    {
                        // Check if the vertex is whitelisted.
                        if (child->getId() == vertex->getId())
                        {
                            return true;
                        }
                        ++it;
                    }
                    else
                    {
                        it = whitelistedChildren_.erase(it);
                    }
                }
                return false;
            }

            void Vertex::blacklistAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                blacklistedChildren_.emplace_back(vertex);
            }

            bool Vertex::isBlacklistedAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                auto it = blacklistedChildren_.begin();
                while (it != blacklistedChildren_.end())
                {
                    // Check if the child is a valid vertex.
                    if (const auto child = it->lock())
                    {
                        // Check if the vertex is whitelisted.
                        if (child->getId() == vertex->getId())
                        {
                            return true;
                        }
                        ++it;
                    }
                    else
                    {
                        it = blacklistedChildren_.erase(it);
                    }
                }
                return false;
            }

            bool Vertex::hasCachedNeighbors() const
            {
                return neighborBatchId_ == batchId_;
            }

            void Vertex::cacheNeighbors(const std::vector<std::shared_ptr<Vertex>> &neighbors) const
            {
                neighbors_.clear();
                neighbors_.insert(neighbors_.begin(), neighbors.begin(), neighbors.end());
                neighborBatchId_ = batchId_;
            }

            const std::vector<std::shared_ptr<Vertex>> Vertex::getNeighbors() const
            {
                if (neighborBatchId_ != batchId_)
                {
                    throw ompl::Exception("Requested neighbors from vertex of outdated approximation.");
                }

                std::vector<std::shared_ptr<Vertex>> neighbors;
                for (const auto &neighbor : neighbors_)
                {
                    assert(neighbor.lock());
                    neighbors.emplace_back(neighbor.lock());
                }

                return neighbors;
            }

            std::vector<std::shared_ptr<Vertex>> Vertex::getForwardChildren() const
            {
                std::vector<std::shared_ptr<Vertex>> children;
                for (const auto &child : forwardChildren_)
                {
                    assert(!child.expired());
                    children.emplace_back(child.lock());
                }
                return children;
            }

            std::vector<std::shared_ptr<Vertex>> Vertex::getReverseChildren() const
            {
                std::vector<std::shared_ptr<Vertex>> children;
                children.reserve(reverseChildren_.size());
                for (const auto &child : reverseChildren_)
                {
                    assert(!child.expired());
                    children.emplace_back(child.lock());
                }
                return children;
            }

            bool Vertex::isConsistent() const
            {
                // Return whether the cost to come is the same now as when this vertex was expanded.
                return objective_->isCostEquivalentTo(costToComeFromGoal_, expandedCostToComeFromGoal_);
            }
            
            void Vertex::setReverseQueuePointer(
                typename ompl::BinaryHeap<
                    std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                    std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                       const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                    Element *pointer)
            {
                reverseQueuePointerId_ = batchId_;
                reverseQueuePointer_ = pointer;
            }

            void Vertex::setForwardQueuePointer(
                typename ompl::BinaryHeap<
                    std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                    std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                       const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                    Element *pointer)
            {
                forwardQueuePointerId_ = batchId_;
                forwardQueuePointer_ = pointer;
            }
            
            typename ompl::BinaryHeap<
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                   const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                Element *
                Vertex::getForwardQueuePointer() const
            {
                if (batchId_ != forwardQueuePointerId_)
                {
                    forwardQueuePointer_ = nullptr;
                }
                return forwardQueuePointer_;
            }


            typename ompl::BinaryHeap<
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                   const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                Element *
                Vertex::getReverseQueuePointer() const
            {
                if (batchId_ != reverseQueuePointerId_)
                {
                    reverseQueuePointer_ = nullptr;
                }
                return reverseQueuePointer_;
            }          

            void Vertex::resetForwardQueuePointer()
            {
                forwardQueuePointer_ = nullptr;
            }
            
            void Vertex::resetReverseQueuePointer()
            {
                reverseQueuePointer_ = nullptr;
            }

        }  // namespace aitstar

    }  // namespace geometric

}  // namespace ompl
