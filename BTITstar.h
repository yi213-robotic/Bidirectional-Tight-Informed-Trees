/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, University of California, Irvine
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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BTITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BTITSTAR_

#include <algorithm>
#include <memory>
#include <unordered_map>
#include "ompl/base/Planner.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/lazyinformedtrees/btitstar/ImplicitGraph.h"
#include "ompl/geometric/planners/lazyinformedtrees/btitstar/Vertex.h"
#include "ompl/geometric/planners/lazyinformedtrees/btitstar/Queuetypes.h"
//#include "linQuad.h"
//#include "DI.h"
#include <chrono>
using namespace std::chrono;
namespace ompl
{
    namespace geometric
    {
        /**
        @anchor gbtitstar

        \ref gbtitstar "BTIT*" (Bidirectional Tight Informed Trees) is an almost-surely asymptotically optimal path planner via provably admisible bidirectional heuristic.
   
        */

        /** \brief Bidirectional Tight Informed Trees (BTIT*) */
        class BTITstar : public ompl::base::Planner
        {
        public:
            /** \brief Constructs a BTIT*. */
            explicit BTITstar(const ompl::base::SpaceInformationPtr &spaceInformation);

            /** \brief Destructs a BTIT*. */
            ~BTITstar() = default;

            /** \brief Additional setup that can only be done once a problem definition is set. */
            void setup() override;

            /** \brief Checks whether the planner is successfully setup. */
            ompl::base::PlannerStatus::StatusType ensureSetup();

            /** \brief Checks whether the problem is successfully setup. */
            ompl::base::PlannerStatus::StatusType
            ensureStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition);

            /** \brief Clears the algorithm's internal state. */
            void clear() override;

            /** \brief Solves a motion planning problem. */
            ompl::base::PlannerStatus
            solve(const ompl::base::PlannerTerminationCondition &terminationCondition) override;

            /** \brief Get the cost of the incumbent solution. */
            ompl::base::Cost bestCost() const;
            /** \brief Get the planner data. */
            void getPlannerData(base::PlannerData &data) const override;
	    
            bool searchTermination();
            bool selectExpansion(bool & forwardDir_);
            /** \brief get the priority C for a vertex*/
	    
            void betterSolution(const std::shared_ptr<btitstar::Vertex> &vertex, ompl::base::Cost meetCost, ompl::base::Cost gCost, ompl::base::Cost &hCost);  
            
            void updateForwardCost(const std::shared_ptr<btitstar::Vertex> &vertex, ompl::base::Cost costToCome, ompl::base::Cost &costToGo);
            void updateReverseCost(const std::shared_ptr<btitstar::Vertex> &vertex, ompl::base::Cost costToCome, ompl::base::Cost &costToGo);
            /** \brief Set the batch size. */
            void setBatchSize(std::size_t batchSize);

            /** \brief Get the batch size. */
            std::size_t getBatchSize() const;
            
            void runTime();

            /** \brief Set the rewire factor of the RGG graph. */
            void setRewireFactor(double rewireFactor);
            
            bool isValid(const btitstar::keyEdgePair &edge, double arriveTime_);
            
            bool couldRefineSolution(ompl::base::Cost fValue_, bool pivotalState_);
            
            bool meetingHueristicDomination();

            /** \brief Get the rewire factor of the RGG graph. */
            double getRewireFactor() const;

            /** \brief Set whether to track approximate solutions. */
            void trackApproximateSolutions(bool track);

            /** \brief Get whether approximate solutions are tracked. */
            bool areApproximateSolutionsTracked() const;

            /** \brief Set whether pruning is enabled or not. */
            void enablePruning(bool prune);

            /** \brief Get whether pruning is enabled or not. */
            bool isPruningEnabled() const;

            /** \brief Set whether to use a k-nearest RGG connection model. If false, AIT* uses an r-disc model. */
            void setUseKNearest(bool useKNearest);

            /** \brief Get whether to use a k-nearest RGG connection model. If false, AIT* uses an r-disc model. */
            bool getUseKNearest() const;

            /** \brief Set the maximum number of goals AIT* will sample from sampleable goal regions. */
            void setMaxNumberOfGoals(unsigned int numberOfGoals);

            ompl::base::Cost MaxValue(const ompl::base::Cost v1, const ompl::base::Cost v2) const;  

            /** \brief Get the maximum number of goals AIT* will sample from sampleable goal regions. */
            unsigned int getMaxNumberOfGoals() const;

            /** \brief Get the edge queue. */
            std::vector<btitstar::Edge> getEdgesInQueue() const;

            /** \brief Get the vertex queue. */
            std::vector<std::shared_ptr<btitstar::Vertex>> getVerticesInQueue() const;

            /** \brief Get the next edge in the queue. */
            //aitstar::Edge getNextEdgeInQueue() const;

            /** \brief Get the next vertex in the queue. */
            std::shared_ptr<btitstar::Vertex> getNextVertexInQueue() const;

            /** \brief Get the vertices in the reverse search tree. */
            std::vector<std::shared_ptr<btitstar::Vertex>> getVerticesInReverseSearchTree() const;
        private:
            /** \brief Performs one iteration of AIT*. */
            void iterate(const ompl::base::PlannerTerminationCondition &terminationCondition);
            
            ompl::base::SpaceInformationPtr spaceInformation_;
            
            ompl::base::State *detectionState_;
                   
            /** \brief Inserts or updates an edge in the forward queue. */
            /** \brief Clears the forward queue, resetting the lookups of the vertices in it. */
            void clearForwardQueue();
    
            /** \brief Clears the reverse queue, resetting the lookups of the vertices in it. */
            void clearReverseQueue();
            void informAboutNewSolution() const;
            void selectingMeetingVertex();
            /** \brief Prints a message using OMPL_INFORM to let the user know of the planner status. */
            void informAboutPlannerStatus(ompl::base::PlannerStatus::StatusType status) const;

            /** \brief Inserts the goal vertices of the graph into the reverse search queue. */
            void insertGoalVerticesInReverseQueue();

            /** \brief Insert start vertices in the forward queue. */
            void expandStartIntoForwardQueue();
            void expandGoalIntobackwardQueue();
            /** \brief Check whether the reverse search must be continued. */
            /** \brief Returns the path a start to the argument. */
            std::shared_ptr<ompl::geometric::PathGeometric>
            getPathToVertex(const std::shared_ptr<btitstar::Vertex> &vertex) const;
            
            ompl::base::Cost costToGoInForwardQueue(const std::shared_ptr<btitstar::Vertex> &vertex, const ompl::base::Cost &costToCome) const;
            ompl::base::Cost costToGoInBackwardQueue(const std::shared_ptr<btitstar::Vertex> &vertex, const ompl::base::Cost &costToCome) const;
            /** \brief Computes the sort key of an edge. */
            std::array<ompl::base::Cost, 3u> computeEstimatedPathCosts(ompl::base::Cost CostToStart, ompl::base::Cost CostToGoal, ompl::base::Cost motionCost) const;
     
            std::array<ompl::base::Cost, 3u> computeSortKey(const std::shared_ptr<btitstar::Vertex> &parent,
                                                            const std::shared_ptr<btitstar::Vertex> &child) const;

            /** \brief Computes the sort key of a vertex. */
            std::array<ompl::base::Cost, 2u> computeSortKey(const std::shared_ptr<btitstar::Vertex> &vertex) const;

            std::array<ompl::base::Cost, 2u> computeEstimatedPathCosts(ompl::base::Cost CostToStart, ompl::base::Cost CostToGoal) const;

            /** \brief Inserts the outgoing edges of a vertex into the forward edge queue. */
            void insertOutgoingEdges(const std::shared_ptr<btitstar::Vertex> &vertex);

            /** \brief Get all outgoing edges of a vertex. */
            std::vector<btitstar::Edge> getOutgoingEdges(const std::shared_ptr<btitstar::Vertex> &vertex) const;
            void insertOrUpdateInForwardQueue(const std::shared_ptr<btitstar::Vertex> &vertex, ompl::base::Cost CostToCome, ompl::base::Cost CostToGoal); 
            void insertOrUpdateInReverseQueue(const std::shared_ptr<btitstar::Vertex> &vertex, ompl::base::Cost CostToCome, ompl::base::Cost CostToGoal); 
            void printReverseStatement(const std::shared_ptr<btitstar::Vertex> &vertex);
            void printForwardStatement(const std::shared_ptr<btitstar::Vertex> &vertex);
            void updateExactSolution();
            void updateCostToGo(const std::shared_ptr<btitstar::Vertex> &vertex, ompl::base::Cost &costToCome, ompl::base::Cost &costToGo);
            void ForwardSearch(const std::shared_ptr<btitstar::Vertex> &vertex);// forward motion tree
            void ReverseSearch(const std::shared_ptr<btitstar::Vertex> &vertex);// backward motion tree    
            /** \brief Checks whether the input vertex is the new best approximate solution and updates the solution in
             * the problem definition if so. **/
            void updateApproximateSolution(const std::shared_ptr<btitstar::Vertex> &vertex);
            std::vector<btitstar::Edge> getOutgoingForwardEdges(const std::shared_ptr<btitstar::Vertex> &vertex, const ompl::base::Cost costToCome) const;
            std::vector<btitstar::Edge> getOutgoingBackwardEdges(const std::shared_ptr<btitstar::Vertex> &vertex, const ompl::base::Cost costToCome) const;
            /** \brief Checks which vertex is the best approximate solution. */
            void updateApproximateSolution();

            /** \brief Updates the exact solution and if AIT* track approximate solutions, it updates it as well. */
            ompl::base::PlannerStatus::StatusType updateSolution();

            /** \brief Updates the exact solution and if AIT* track approximate solutions, it updates it as well. */
            ompl::base::PlannerStatus::StatusType updateSolution(const std::shared_ptr<btitstar::Vertex> &vertex);

            /** \brief Returns the best cost-to-go-heuristic to any start in the graph. */
            ompl::base::Cost computeCostToGoToStartHeuristic(const std::shared_ptr<btitstar::Vertex> &vertex) const;
            ompl::base::Cost computeCostToGoToGoalHeuristic(const std::shared_ptr<btitstar::Vertex> &vertex) const;

            /** \brief Returns the best cost to go to any goal in the graph. */
            ompl::base::Cost computeCostToGoToGoal(const std::shared_ptr<btitstar::Vertex> &vertex) const;
            
            /** \brief addming more samples*/
            bool NeedMoreSamples();

            /** \brief Returns the best cost to come form the goal of any start. */
            ompl::base::Cost computeBestCostToComeFromGoalOfAnyStart() const;

            /** \brief Counts the number of vertices in the forward tree. */
            std::size_t countNumVerticesInForwardTree() const;

            /** \brief Counts the number of vertices in the reverse tree. */
            std::size_t countNumVerticesInReverseTree() const;

            /** \brief Sets the cost to come from to goal of the reverse search to infinity for the whole branch.
             */
            void invalidateCostToComeFromGoalOfReverseBranch(const std::shared_ptr<btitstar::Vertex> &vertex);

            /** \brief The cost of the incumbent solution. */
            ompl::base::Cost solutionCost_;

            mutable ompl::base::Cost samplingBound_;

            /** \brief The optimal solution found so far for a give RGG. */          
            ompl::base::Cost C_best;
            
            /** \brief f,g,h value in both motion tree. */  
            std::array<ompl::base::Cost, 3u> forwardEValue;
            std::array<ompl::base::Cost, 3u> backwardEValue;
            /** \brief The cost to come to the vertex that is closest to the goal (in cost space). */
            ompl::base::Cost approximateSolutionCost_{};

            /** \brief The cost to go to the goal from the current best approximate solution. */
            ompl::base::Cost approximateSolutionCostToGoal_{};
            ompl::base::Cost boundedValue_{0u};
            
            ompl::base::Cost meetGValue_{0u};
            
            ompl::base::Cost meetHValue_{0u};
            
            ompl::base::Cost currGValue_{0u};
            
            ompl::base::Cost currHValue_{0u};
            
            ompl::base::Cost f_min{0u};
            
            ompl::base::Cost f_max{0u};
            
            ompl::base::Cost f_minf{0u};
            
            ompl::base::Cost f_minb{0u};
            
            ompl::base::Cost loc_fmin{0u};
            
            ompl::base::Cost meetValue{0u};
            
            /** \brief The increasingly dense sampling-based approximation. */
            btitstar::ImplicitGraph graph_;

            /** \brief The forward motion tree. */
            btitstar::VertexQueue forwardQueue_;
            
            /** \brief The reverse motion tree. */
            btitstar::VertexQueue reverseQueue_;      

            /** \biref the best vertex*/
            std::shared_ptr<btitstar::Vertex> BestVertex_;
            
            std::shared_ptr<ompl::geometric::PathGeometric> path_;
            
            /** \biref the middler vertex*/
            btitstar::MiddleVertex V_meet;  

            /** \brief Lexicographically compares the keys of two edges. */
            bool isVertexBetter(const btitstar::KeyVertexPair &lhs, const btitstar::KeyVertexPair &rhs) const;

            /** \brief the number of edges in the forward queue with inconsistent target states. */
            std::size_t numInconsistentOrUnconnectedTargets_{0u};

            /** \brief The number of iterations that have been performed. */
            std::size_t numIterations_{0u};
            std::size_t numSparseCollisionChecksCurrentLevel_{1u};
            /** \brief The number of samples per batch. */
            std::size_t batchSize_{50u};
            /** \brief The option that specifies whether to track approximate solutions. */
            bool trackApproximateSolutions_{true};
            /** \brief The option that specifies whether to prune the graph of useless samples. */
            bool isPruningEnabled_{false};
            bool increProcess_{false};
            bool firstIntersection_{true};
            void showingRunningtime();
            bool betterCost(const auto & cost1, const auto & cost2);
            bool findASolution_{false};
            
            bool newSolution_{false};
            
            bool overBound_{false};
            
            bool canTerminate_{false};
            
            bool betterForward_{false};
            
            bool betterReverse_{false};
            
            bool goalCloseToStart_{false};
            bool need_Prune_{false};
            double time_taken{0u};
            high_resolution_clock::time_point  startT,endT;
            /** \brief Syntactic helper to get at the optimization objective of the planner base class. */
            ompl::base::OptimizationObjectivePtr objective_;

            /** \brief Syntactic helper to get at the motion validator of the planner base class. */
            ompl::base::MotionValidatorPtr motionValidator_;

            /** \brief The number of processed edges. */
            std::size_t numProcessedEdges_{0u};
            std::shared_ptr<ompl::base::StateSpace> space_;
            /** \brief The number of edge collision checks performed. */
            std::size_t numEdgeCollisionChecks_{0u};
        };
    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_LAZYINFORMEDTREES_BTITSTAR_
