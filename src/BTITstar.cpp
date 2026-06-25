/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, University of New Hampshire
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

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"
#include "ompl/geometric/planners/lazyinformedtrees/BTITstar.h"

#include <algorithm>
#include <cmath>
#include <string>

#include <boost/range/adaptor/reversed.hpp>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/util/Console.h"

using namespace std::string_literals;
using namespace ompl::geometric::btitstar;
namespace ob = ompl::base;
using namespace std;
namespace ompl
{
    namespace geometric
    {
        BTITstar::BTITstar(const ompl::base::SpaceInformationPtr &spaceInformation)
          : ompl::base::Planner(spaceInformation, "BTITstar")
          , solutionCost_()
          , graph_(solutionCost_)
          , detectionState_(spaceInformation->allocState())
          , space_(spaceInformation->getStateSpace())
          , forwardQueue_([this](const auto &lhs, const auto &rhs) { return isVertexBetter(lhs, rhs); })
          , reverseQueue_([this](const auto &lhs, const auto &rhs) { return isVertexBetter(lhs, rhs); })
        {
            // Specify BTIT*'s planner specs.
            specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
            specs_.multithreaded = false;
            specs_.approximateSolutions = true;
            specs_.optimizingPaths = true;
            specs_.directed = true;
            specs_.provingSolutionNonExistence = false;
            specs_.canReportIntermediateSolutions = true;
            spaceInformation_ = spaceInformation;
            // Register the setting callbacks.
            declareParam<bool>("use_k_nearest", this, &BTITstar::setUseKNearest, &BTITstar::getUseKNearest, "0,1");
            declareParam<double>("rewire_factor", this, &BTITstar::setRewireFactor, &BTITstar::getRewireFactor,
                                 "1.0:0.01:3.0");
            declareParam<std::size_t>("samples_per_batch", this, &BTITstar::setBatchSize, &BTITstar::getBatchSize,
                                      "1:1:1000");
            declareParam<bool>("use_graph_pruning", this, &BTITstar::enablePruning, &BTITstar::isPruningEnabled, "0,1");
            declareParam<bool>("find_approximate_solutions", this, &BTITstar::trackApproximateSolutions,
                               &BTITstar::areApproximateSolutionsTracked, "0,1");
            declareParam<std::size_t>("set_max_num_goals", this, &BTITstar::setMaxNumberOfGoals,
                                      &BTITstar::getMaxNumberOfGoals, "1:1:1000");

            // Register the progress properties.
            addPlannerProgressProperty("iterations INTEGER", [this]() { return std::to_string(numIterations_); });
            addPlannerProgressProperty("best cost DOUBLE", [this]() { return std::to_string(solutionCost_.value()); });
            addPlannerProgressProperty("state collision checks INTEGER",
                                       [this]() { return std::to_string(graph_.getNumberOfStateCollisionChecks()); });
            addPlannerProgressProperty("edge collision checks INTEGER",
                                       [this]() { return std::to_string(numEdgeCollisionChecks_); });
            addPlannerProgressProperty("nearest neighbour calls INTEGER",
                                       [this]() { return std::to_string(graph_.getNumberOfNearestNeighborCalls()); });
        }
        
        void BTITstar::setup()
        {
            // Call the base-class setup.
            Planner::setup();

            // Check that a problem definition has been set.
            if (static_cast<bool>(Planner::pdef_))
            {
                // Default to path length optimization objective if none has been specified.
                if (!pdef_->hasOptimizationObjective())
                {
                    OMPL_WARN("%s: No optimization objective has been specified. Defaulting to path length.",
                              Planner::getName().c_str());
                    Planner::pdef_->setOptimizationObjective(
                        std::make_shared<ompl::base::PathLengthOptimizationObjective>(Planner::si_));
                }

                if (static_cast<bool>(pdef_->getGoal()))
                {
                    // If we were given a goal, make sure its of appropriate type.
                    if (!(pdef_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION)))
                    {
                        OMPL_ERROR("BTIT* is currently only implemented for goals that can be cast to "
                                   "ompl::base::GOAL_SAMPLEABLE_GOAL_REGION.");
                        setup_ = false;
                        return;
                    }
                }

                // Pull the optimization objective through the problem definition.
                objective_ = pdef_->getOptimizationObjective();

                // Initialize the solution cost to be infinite.
                samplingBound_ = C_best = solutionCost_ = objective_->infiniteCost();
                boundedValue_= objective_->infiniteCost();
                approximateSolutionCost_ = objective_->infiniteCost();
                approximateSolutionCostToGoal_ = objective_->infiniteCost();

                // Pull the motion validator through the space information.
                motionValidator_ = si_->getMotionValidator();

                // Setup a graph.
                graph_.setup(si_, pdef_, &pis_); 
            }
            else
            {
                // AIT* can't be setup without a problem definition.
                setup_ = false;
                OMPL_WARN("BFIT*: Unable to setup without a problem definition.");
            }
        }

        ompl::base::PlannerStatus::StatusType BTITstar::ensureSetup()
        {
            // Call the base planners validity check. This checks if the
            // planner is setup if not then it calls setup().
            checkValidity();

            // Ensure the planner is setup.
            if (!setup_)
            {
                OMPL_ERROR("%s: The planner is not setup.", name_.c_str());
                return ompl::base::PlannerStatus::StatusType::ABORT;
            }

            // Ensure the space is setup.
            if (!si_->isSetup())
            {
                OMPL_ERROR("%s: The space information is not setup.", name_.c_str());
                return ompl::base::PlannerStatus::StatusType::ABORT;
            }

            return ompl::base::PlannerStatus::StatusType::UNKNOWN;
        }
        
        std::size_t BTITstar::countNumVerticesInReverseTree() const
        {
            std::size_t numVerticesInReverseTree = 0u;
            auto vertices = graph_.getVertices();
            for (const auto &vertex : vertices)
            {
                if (graph_.isGoal(vertex) || vertex->hasReverseParent())
                {
                    ++numVerticesInReverseTree;
                }
            }
            return numVerticesInReverseTree;
        }
         
        std::size_t BTITstar::countNumVerticesInForwardTree() const
        {
            std::size_t numVerticesInForwardTree = 0u;
            auto vertices = graph_.getVertices();
            for (const auto &vertex : vertices)
            {
                if (graph_.isStart(vertex) || vertex->hasForwardParent())
                {
                    ++numVerticesInForwardTree;
                }
            }
            return numVerticesInForwardTree;
        }  

        ompl::base::PlannerStatus::StatusType
        BTITstar::ensureStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // If the graph currently does not have a start state, try to get one.
            if (!graph_.hasAStartState())
            {
                graph_.updateStartAndGoalStates(terminationCondition, &pis_);

                // If we could not get a start state, then there's nothing to solve.
                if (!graph_.hasAStartState())
                {
                    OMPL_WARN("%s: No solution can be found as no start states are available", name_.c_str());
                    return ompl::base::PlannerStatus::StatusType::INVALID_START;
                }
            }

            // If the graph currently does not have a goal state, we wait until we get one.
            if (!graph_.hasAGoalState())
            {
                graph_.updateStartAndGoalStates(terminationCondition, &pis_);

                // If the graph still doesn't have a goal after waiting, then there's nothing to solve.
                if (!graph_.hasAGoalState())
                {
                    OMPL_WARN("%s: No solution can be found as no goal states are available", name_.c_str());
                    return ompl::base::PlannerStatus::StatusType::INVALID_GOAL;
                }
            }

            // Would it be worth implementing a 'setup' or 'checked' status type?
            return ompl::base::PlannerStatus::StatusType::UNKNOWN;
        }

        void BTITstar::clear()
        {
            graph_.clear();
            forwardQueue_.clear();
            reverseQueue_.clear();
            solutionCost_ = objective_->infiniteCost();
            approximateSolutionCost_ = objective_->infiniteCost();
            approximateSolutionCostToGoal_ = objective_->infiniteCost();
            numIterations_ = 0u;
            numInconsistentOrUnconnectedTargets_ = 0u;
            Planner::clear();
            setup_ = false;
        }

        ompl::base::PlannerStatus BTITstar::solve(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Ensure that the planner and state space are setup before solving.
            auto status = ensureSetup();

            // Return early if the planner or state space are not setup.
            if (status == ompl::base::PlannerStatus::StatusType::ABORT)
            {
                return status;
            }

            // Ensure that the problem has start and goal states before solving.
            status = ensureStartAndGoalStates(terminationCondition);

            // Return early if the problem cannot be solved.
            if (status == ompl::base::PlannerStatus::StatusType::INVALID_START ||
                status == ompl::base::PlannerStatus::StatusType::INVALID_GOAL)
            {
                return status;
            }
            OMPL_INFORM("%s: Solving the given planning problem. The current best solution cost is %.4f", name_.c_str(),
                        solutionCost_.value());
                
            // Iterate to solve the problem.
            startT = chrono::high_resolution_clock::now();
            while (!terminationCondition && !objective_->isSatisfied(solutionCost_))
            { //cerr << terminationCondition->timed
                iterate(terminationCondition);
               
            } cerr << "---- " << solutionCost_<< endl;
              cout << solutionCost_ << "  " << 10.000000000<< endl;
            exit(-1);
            // Someone might call ProblemDefinition::clearSolutionPaths() between invocations of Planner::sovle(), in
            // which case previously found solutions are not registered with the problem definition anymore.
            status = updateSolution();

            // Let the caller know the status.
            informAboutPlannerStatus(status);
            return status;
        }

        ompl::base::Cost BTITstar::bestCost() const
        {
            return solutionCost_;
        }


        void BTITstar::informAboutNewSolution() const
        {
            OMPL_INFORM("%s (%u iterations): Found a new exact solution of cost %.4f. Sampled a total of %u states, %u "
                        "of which were valid samples (%.1f \%). Processed %u edges, %u of which were collision checked "
                        "(%.1f \%). The forward search tree has %u vertices, %u of which are start states. The reverse "
                        "search tree has %u vertices, %u of which are goal states.",
                        name_.c_str(), numIterations_, solutionCost_.value(), graph_.getNumberOfSampledStates(),
                        graph_.getNumberOfValidSamples(),
                        graph_.getNumberOfSampledStates() == 0u ?
                            0.0 :
                            100.0 * (static_cast<double>(graph_.getNumberOfValidSamples()) /
                                     static_cast<double>(graph_.getNumberOfSampledStates())),
                        numProcessedEdges_, numEdgeCollisionChecks_,
                        numProcessedEdges_ == 0u ? 0.0 :
                                                   100.0 * (static_cast<float>(numEdgeCollisionChecks_) /
                                                            static_cast<float>(numProcessedEdges_)),
                        countNumVerticesInForwardTree(), graph_.getStartVertices().size(),
                        countNumVerticesInReverseTree(), graph_.getGoalVertices().size());
        }
       
        void BTITstar::informAboutPlannerStatus(ompl::base::PlannerStatus::StatusType status) const
        {
            switch (status)
            {
                case ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION:
                {
                    OMPL_INFORM("%s (%u iterations): Found an exact solution of cost %.4f.", name_.c_str(),
                                numIterations_, solutionCost_.value());
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION:
                {
                    OMPL_INFORM("%s (%u iterations): Did not find an exact solution, but found an approximate "
                                "solution "
                                "of cost %.4f which is %.4f away from a goal (in cost space).",
                                name_.c_str(), numIterations_, approximateSolutionCost_.value(),
                                approximateSolutionCostToGoal_.value());
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::TIMEOUT:
                {
                    if (trackApproximateSolutions_)
                    {
                        OMPL_INFORM("%s (%u iterations): Did not find any solution.", name_.c_str(), numIterations_);
                    }
                    else
                    {
                        OMPL_INFORM("%s (%u iterations): Did not find an exact solution, and tracking approximate "
                                    "solutions is disabled.",
                                    name_.c_str(), numIterations_);
                    }
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::INFEASIBLE:
                case ompl::base::PlannerStatus::StatusType::UNKNOWN:
                case ompl::base::PlannerStatus::StatusType::INVALID_START:
                case ompl::base::PlannerStatus::StatusType::INVALID_GOAL:
                case ompl::base::PlannerStatus::StatusType::UNRECOGNIZED_GOAL_TYPE:
                case ompl::base::PlannerStatus::StatusType::CRASH:
                case ompl::base::PlannerStatus::StatusType::ABORT:
                case ompl::base::PlannerStatus::StatusType::TYPE_COUNT:
                {
                    OMPL_INFORM("%s (%u iterations): Unable to solve the given planning problem.", name_.c_str(),
                                numIterations_);
                }
            }

            OMPL_INFORM(
                "%s (%u iterations): Sampled a total of %u states, %u of which were valid samples (%.1f \%). "
                "Processed %u edges, %u of which were collision checked (%.1f \%). The forward search tree "
                "has %u vertices. The reverse search tree has %u vertices.",
                name_.c_str(), numIterations_, graph_.getNumberOfSampledStates(), graph_.getNumberOfValidSamples(),
                graph_.getNumberOfSampledStates() == 0u ?
                    0.0 :
                    100.0 * (static_cast<double>(graph_.getNumberOfValidSamples()) /
                             static_cast<double>(graph_.getNumberOfSampledStates())),
                numProcessedEdges_, numEdgeCollisionChecks_,
                numProcessedEdges_ == 0u ?
                    0.0 :
                    100.0 * (static_cast<float>(numEdgeCollisionChecks_) / static_cast<float>(numProcessedEdges_)),
                countNumVerticesInForwardTree(), countNumVerticesInReverseTree());
        }

        void BTITstar::getPlannerData(base::PlannerData &data) const
        {
            // base::PlannerDataVertex takes a raw pointer to a state. I want to guarantee, that the state lives as
            // long as the program lives.
            static std::set<std::shared_ptr<Vertex>,
                            std::function<bool(const std::shared_ptr<Vertex> &, const std::shared_ptr<Vertex> &)>>
                liveStates([](const auto &lhs, const auto &rhs) { return lhs->getId() < rhs->getId(); });

            // Fill the planner progress properties.
            Planner::getPlannerData(data);

            // Get the vertices.
            auto vertices = graph_.getVertices();

            // Add the vertices and edges.
            for (const auto &vertex : vertices)
            {
                // Add the vertex to the live states.
                liveStates.insert(vertex);

                // Add the vertex as the right kind of vertex.
                if (graph_.isStart(vertex))
                {
                    data.addStartVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }
                else if (graph_.isGoal(vertex))
                {
                    data.addGoalVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }
                else
                {
                    data.addVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }

                // If it has a parent, add the corresponding edge.
                if (vertex->hasForwardParent())
                {
                    data.addEdge(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()),
                                 ompl::base::PlannerDataVertex(vertex->getForwardParent()->getState(),
                                                               vertex->getForwardParent()->getId()));
                }
            }
        }

        void BTITstar::setBatchSize(std::size_t batchSize)
        {
            batchSize_ = batchSize;
        }

        std::size_t BTITstar::getBatchSize() const
        {
            return batchSize_;
        }

        void BTITstar::setRewireFactor(double rewireFactor)
        {
            graph_.setRewireFactor(rewireFactor);
        }

        double BTITstar::getRewireFactor() const
        {
            return graph_.getRewireFactor();
        }

        void BTITstar::trackApproximateSolutions(bool track)
        {
            trackApproximateSolutions_ = track;
            if (!trackApproximateSolutions_)
            {
                if (static_cast<bool>(objective_))
                {
                    approximateSolutionCost_ = objective_->infiniteCost();
                    approximateSolutionCostToGoal_ = objective_->infiniteCost();
                }
            }
        }

        bool BTITstar::areApproximateSolutionsTracked() const
        {
            return trackApproximateSolutions_;
        }

        void BTITstar::enablePruning(bool prune)
        {
            isPruningEnabled_ = prune;
        }

        bool BTITstar::isPruningEnabled() const
        {
            return isPruningEnabled_;
        }

        void BTITstar::setUseKNearest(bool useKNearest)
        {
            graph_.setUseKNearest(useKNearest);
        }

        bool BTITstar::getUseKNearest() const
        {
            return graph_.getUseKNearest();
        }

       void BTITstar::setMaxNumberOfGoals(unsigned int numberOfGoals)
        {
            graph_.setMaxNumberOfGoals(numberOfGoals);
        }

        unsigned int BTITstar::getMaxNumberOfGoals() const
        {
            return graph_.getMaxNumberOfGoals();
        }

        void BTITstar::clearReverseQueue()
        {
            reverseQueue_.clear();
        }
        void BTITstar::clearForwardQueue()
        {
            forwardQueue_.clear();
        }

        void BTITstar::updateExactSolution()
        {       
       
                    // Create a solution.
                    ompl::base::PlannerSolution solution(path_);
                    solution.setPlannerName(name_);

                    // Set the optimized flag.
                    solution.setOptimized(objective_, solutionCost_, objective_->isSatisfied(solutionCost_));

                    // Let the problem definition know that a new solution exists.
                    pdef_->addSolutionPath(solution);

                    // Let the user know about the new solution.
                    informAboutNewSolution();
            
        }

        ompl::base::PlannerStatus::StatusType BTITstar::updateSolution()
        {
            updateExactSolution();
            
            if (objective_->isFinite(solutionCost_))
            {
                return ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION;
            }
            else if (trackApproximateSolutions_)
            {
                //updateApproximateSolution();
                return ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION;
            }
            else
            {
                return ompl::base::PlannerStatus::StatusType::TIMEOUT;
            }
        }
     
         void BTITstar::expandStartIntoForwardQueue()
        {
            for (const auto &start : graph_.getStartVertices())
            {
                start->setCostToComeFromStart(objective_->identityCost());
                start->setStart();
                start->resetForwardExpanded();
                insertOrUpdateInForwardQueue(start, objective_->identityCost(), start->getLowerCostBoundToGoal());
            }
        }
    

        void BTITstar::expandGoalIntobackwardQueue()
        {
            for (const auto &goal : graph_.getGoalVertices())
            {
                goal->setCostToComeFromGoal(objective_->identityCost());
                goal->setGoal();
                goal->resetReverseExpanded();
                insertOrUpdateInReverseQueue(goal, objective_->identityCost(), goal->getLowerCostBoundToStart());
            }
        }

        
        bool BTITstar::NeedMoreSamples()
        {
           return reverseQueue_.empty() || forwardQueue_.empty();        
        }
                                   
        bool BTITstar::searchTermination()
        {
             if(findASolution_)
             {  
                    if(firstIntersection_)
                     {  
                         firstIntersection_ = false;
                         return true; 
                     }
		     if(!objective_->isCostLargerThan(C_best,f_min))
		     {  return true; } 
		     
		     if(BestVertex_->pivotalState() && !objective_->isCostLargerThan(C_best,f_max))
		     {  return true; }
		     
		     if(canTerminate_) //TC3-TC6
		     {
		         return true;
		     }      
             }
             return false;
        }
        
        void BTITstar::runTime()
        {
            endT = chrono::high_resolution_clock::now();
            time_taken = chrono::duration_cast<chrono::nanoseconds>(endT - startT).count();
            time_taken *= 1e-9;
        }
         
        void BTITstar::showingRunningtime() 
        {
            cerr  << fixed << time_taken << setprecision(9) << endl;
            cout  << fixed << time_taken << setprecision(9) << endl;
        } 
                     
        void BTITstar::iterate(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // If it needs more samples to imporve the solution, do it now. 
            if(NeedMoreSamples())
            {         
              // Add new samples to the graph, respecting the termination condition.
                if(graph_.addSamples(batchSize_, terminationCondition,need_Prune_,samplingBound_))
                {
                    //Record the current best solution found so far
                    C_best = solutionCost_;    
                    // Mark the search that has not found a better solution yet 
                    need_Prune_ =findASolution_ = false;  
                    
                    // Starting a new search with expanding start and goal vertices simultaneously
                    expandStartIntoForwardQueue();
                    expandGoalIntobackwardQueue();
                }
            } 
            
            // Select the state with minimum priority in both search directions for expansion
            bool forwardDir_ = false;
            if(!selectExpansion(forwardDir_))
            { return;      }
            
            // Continue the operation, if no termination condition is triggered. 
            if(!searchTermination()) 
            {
                canTerminate_ = newSolution_ = false;
                loc_fmin = objective_->infiniteCost();
                if(forwardDir_) 
                {    
                    BestVertex_->setForwardExpanded();
                    ForwardSearch(BestVertex_);  
                }
                else
                {    
                    BestVertex_->setReverseExpanded(); 
                    ReverseSearch(BestVertex_); 
                }
                
                if(newSolution_)
                {
                   findASolution_ = newSolution_;
                   betterForward_ = betterReverse_ = false;
                   meetGValue_ = currGValue_;
                   meetHValue_ = currHValue_;
                }
                else
                {
                   if(findASolution_)
                   {
                        bool betterSolu_ = forwardDir_ ? !betterForward_ : !betterReverse_;
                        //TC3
                        if(overBound_ && betterSolu_)
                        {
                           canTerminate_ =  meetingHueristicDomination() || objective_->isCostBetterThan(loc_fmin, C_best);
                        }
                        //TC4-TC6
                        if(!overBound_ && betterSolu_)
                        {
                           canTerminate_ =  meetingHueristicDomination() || objective_->isCostBetterThan(loc_fmin, C_best);  
                        }
                        else
                        {
                            canTerminate_ = BestVertex_->postSoluState() && objective_->isCostBetterThan(loc_fmin, C_best);
                        }
                   }
                }
            } 
            else //Otherwise, prepare to restart a new search with additional new samples and cull states that cannot improve C_best  
            {
                               
                                if(objective_->isCostBetterThan(C_best,solutionCost_))
                                { 
                                    solutionCost_ = C_best; 
                                    need_Prune_ = true;
                                    runTime();
                                    cerr << solutionCost_ << "  ";
                                    cout << solutionCost_ << "  ";
                                    showingRunningtime();
                                    path_ = getPathToVertex(V_meet.second);
                                }                                 
                                clearReverseQueue();
                                clearForwardQueue();
                             
            } 
            // Keep track of the number of iterations.
            ++numIterations_;
        }
        
        bool BTITstar::meetingHueristicDomination()
        {
             return !objective_->isCostBetterThan(currGValue_,meetGValue_) && !objective_->isCostBetterThan(currHValue_,meetHValue_);
        }
        
        bool BTITstar::betterCost(const auto & cost1, const auto & cost2)
        {
            return objective_->isCostBetterThan(cost1,cost2);
        }  

        void BTITstar::printForwardStatement(const std::shared_ptr<btitstar::Vertex> &vertex)
        {
                 if(vertex->isStart())
                 {
                     cerr << "vertex-> " << vertex->getId() << " is expanding in forward preliminary tree. "   << " " << vertex->getCostToComeFromStart() << "  " << vertex->getLowerCostBoundToGoal() << endl;
                 }
                 else
                 {
                    ompl::base::Cost ecost(2.9); 
                    if(objective_->isCostBetterThan(ecost,solutionCost_))
                    cerr << "vertex-> " << vertex->getId() << " is expanding in forward preliminary tree. "  << "  "  << vertex->getForwardParent()->getId() << " " << vertex->getCostToComeFromStart() << "  " << f_minf << " " << vertex->hasReverseParent() << endl;
                 }
                 
        }  
        
        bool BTITstar::couldRefineSolution(ompl::base::Cost fValue_, bool pivotalState_)
        {
                if(!overBound_ || !pivotalState_)  
                    return objective_->isCostBetterThan(fValue_,C_best);
                return  objective_->isCostBetterThan(fValue_,meetValue);
        }
        
        void BTITstar::ForwardSearch(const std::shared_ptr<btitstar::Vertex> &vertex) 
        {   
            //printForwardStatement(vertex);
            for (const auto &child : vertex->getForwardChildren())
            {      
                    if(findASolution_)
                    {
                       child->setPostSoluState();
                    } 
                    if(vertex->hasReverseParent() && vertex->getReverseParent()->getId() == child->getId())
                    {     continue;    }


                    auto edgeCost = child->getForwardEdgeCost();
                    // g_hat(x_v) 
                    auto gValue =  objective_->combineCosts(vertex->getCostToComeFromStart(),edgeCost);

                    // h_bar(x_v): lower cost bound to go such as Eculidean distance 
                    auto hValue = child->getLowerCostBoundToGoal();     
                         child->setCostToComeFromStart(gValue); 
                         gValue = child->getCostToComeFromStart();
                         updateForwardCost(child, gValue, hValue);
                    ompl::base::Cost est_fValue = objective_->combineCosts(gValue,hValue);
                    if(findASolution_)
                    {
                        if(objective_->isCostBetterThan(est_fValue,loc_fmin))
                        {
                              loc_fmin = est_fValue; 
                        }
                        est_fValue = (overBound_ ? est_fValue: objective_->combineCosts(gValue,gValue));
                        betterForward_ = couldRefineSolution(est_fValue, child->pivotalState());
                    }     
                         //cerr << " old child in forward tree " << child->getId() << " g-value: " << gValue << ", f-value: " << objective_->combineCosts(gValue,hValue)  << endl;                    
                    if(!child->hasReverseParent() && objective_->isCostLargerThan(objective_->combineCosts(gValue,gValue),solutionCost_))
                    {      continue;   }
                                        
                    insertOrUpdateInForwardQueue(child,gValue,hValue);            
            }
            
            // Insert new neighbors in to forward queue
            for (const auto &neighbor : graph_.getNeighbors(vertex))
            {    
                // The neighbor cannot be itself
                if(neighbor->getId() == vertex->getId())
                   continue; 
                
                // The neighbor has not been expanded yet.    
                if (!neighbor->forwardExpanded())
                { 
                    // The neighbor should not be the current vertex's reverse parent
                    if(vertex->hasReverseParent() && vertex->getReverseParent()->getId() == neighbor->getId())
                    {    continue;  }
                    
                    // The neighbor should not be the current vertex's forward parent
                    if(neighbor->hasForwardParent() && neighbor->getForwardParent()->getId() == vertex->getId())
                    {    continue;  }
                    
                    // The neighbor should not offer an invalid edge for the vertex
                    if (neighbor->isBlacklistedAsChild(vertex) || vertex->isBlacklistedAsChild(neighbor))
                    {    continue;  }
                    
                    // Get the g-value of the vertex
                    auto gValue = (neighbor->hasForwardParent())? neighbor->getCostToComeFromStart() : objective_->infiniteCost();
                    // c_hat(x_u,x_v)
                    //double arrTime_ = estimatedTimeDDI(vertex->getState(),  neighbor->getState());
                    //ompl::base::Cost edgeCost = estimatedCostDDI(vertex->getState(),  neighbor->getState(), arrTime_);
                    
                    // Get the edge cost between the vertex and its neighbor
                    //auto edgeCost = objective_->motionCostHeuristic(neighbor->getState(), vertex->getState());//auto edgeCost = TrueEdgeCost(neighbor->getState(),vertex->getState());////lqEdgeCost(vertex->getState(),neighbor->getState(), arrTime_)
                    //double arrTime_ = 0;
                    
                    double arrTime_ = estimatedTimeMGLQ(vertex->getState(),neighbor->getState());
                    auto edgeCost = lqEdgeCostMGLQ(vertex->getState(),neighbor->getState(), arrTime_);
                    
                    // g_hat(x_u) + c_hat(x_u,x_v)
                    auto est_gValue = objective_->combineCosts(vertex->getCostToComeFromStart(), edgeCost);
                    // epsilon = min(epsilon,c_hat(x_u,x_v))
                     
                    // If g_F(x_v) > g_F(x_u) + c_hat(x_u,x_v) 
                    if (objective_->isCostBetterThan(est_gValue, gValue)&& !neighbor->isGoal() )
                    {
                         /*if(vertex->getId() == 372 )
                         cerr << neighbor->getId() << "*********************************************************************************************************** " << est_gValue << " " << gValue << "  " << edgeCost << endl;*/
                         if(!neighbor->hasReverseParent() && objective_->isCostLargerThan(objective_->combineCosts(est_gValue,est_gValue),solutionCost_))
                         {      continue;   }
                          
                         //cerr << "hello " << endl;
                         // Set x_u to be x_v's parent and update g_hat(x_v)
                         if(!isValid(make_pair(vertex,neighbor),arrTime_)) 
                         {  
                                continue;
                         }
                         neighbor->setCostToComeFromStart(est_gValue);
                         neighbor->setForwardParent(vertex,edgeCost,arrTime_);  
                         vertex->addToForwardChildren(neighbor);
                         auto hValue = neighbor->getLowerCostBoundToGoal();

                         // Push x_v into forward vertex queue
                         // If x_v is in Q_B,  Uv = min(U,g_f(s) + g_b(s))
                         bool onReverseSearch = neighbor->getReverseQueuePointer() || neighbor->hasReverseParent(); 
                         if(onReverseSearch)//|| 
                         {       
                            keyEdgePair rEdge = make_pair(neighbor,neighbor->getReverseParent());                             
                            auto sumCost_ = objective_->combineCosts(est_gValue,neighbor->getCostToComeFromGoal());
                            //arrTime_,neighbor->getReverseEdgeTime(),pathLengthGL(vertex->getState(),neighbor->getState()).value(),pathLengthGL(neighbor->getState(),neighbor->getReverseParent()->getState()).value() 
                            //cerr <<  sumCost_<< "--------- 3, " << C_best << endl;
                            if(objective_->isCostBetterThan(sumCost_,C_best) && isValid(rEdge,neighbor->getReverseEdgeTime()))//
                            {        
                                     overBound_ = objective_->isCostLargerThan(est_gValue, neighbor->getCostToComeFromGoal()) ? true : false; 
                                     if(overBound_)
                                     {
                                        meetValue = objective_->combineCosts(est_gValue, est_gValue);
                                     } 
                                     betterSolution(neighbor,sumCost_,est_gValue,hValue);
                                     if(firstIntersection_)
                                     {  break; }
                                     ompl::base::Cost est_fValue = objective_->combineCosts(est_gValue, hValue);
                                     if(objective_->isCostBetterThan(est_fValue,loc_fmin))
                                     {
                                        loc_fmin = est_fValue; 
                                     }
                                     insertOrUpdateInForwardQueue(neighbor,est_gValue,hValue); 
                                     continue;
                            }
                         }
                         updateCostToGo(neighbor,est_gValue, hValue); 
                         ompl::base::Cost est_fValue = objective_->combineCosts(est_gValue, hValue);
                         if(findASolution_)
                         {    
                              neighbor->setPostSoluState();
                              if(objective_->isCostBetterThan(est_fValue,loc_fmin))
                              {
                                 loc_fmin = est_fValue; 
                              }
                              est_fValue = (overBound_ ? est_fValue: objective_->combineCosts(est_gValue,est_gValue));
                              betterForward_ = couldRefineSolution(est_fValue, neighbor->pivotalState());
                         }                        
                         if(objective_->isCostBetterThan(est_fValue, solutionCost_))   
                         insertOrUpdateInForwardQueue(neighbor,est_gValue,hValue);  
                         //cerr << "\t \t the new neighbor in forward is:  " << neighbor->getId()  << " f-value: " << objective_->combineCosts(est_gValue,hValue) <<"  " << est_gValue <<  endl; 
                    } 
                }
            }
            
            //cerr << "\t testing here--------------------------------------> " << minimalneighbor_ << endl;      
            if(vertex->hasReverseParent())
            {
                  auto reverseParent = vertex->getReverseParent();
                  auto edgeCost = vertex->getReverseEdgeCost();//objective_->motionCostHeuristic(reverseParent->getState(), vertex->getState())
                  auto currentValue = objective_->combineCosts(vertex->getCostToComeFromStart(),edgeCost);
                  auto hValue = reverseParent->getLowerCostBoundToGoal();
                  reverseParent->setCostToComeFromStart(currentValue);  
                  if(objective_->isCostBetterThan(currentValue,reverseParent->getCostToComeFromStart()))
                  {       
                         auto arriveTime_ = vertex->getReverseEdgeTime();
                         reverseParent->setForwardParent(vertex,edgeCost,arriveTime_);
                         vertex->resetReverseParent();
                         vertex->addToForwardChildren(reverseParent);            
                         updateForwardCost(reverseParent, reverseParent->getCostToComeFromStart(), hValue);
                         insertOrUpdateInForwardQueue(reverseParent,reverseParent->getCostToComeFromStart(),hValue); 
                  }

            }
        }

        void BTITstar::updateForwardCost(const std::shared_ptr<btitstar::Vertex> &vertex, ompl::base::Cost costToCome, ompl::base::Cost &costToGo)
        {
                         vertex->resetForwardExpanded();
                         if(vertex->hasReverseParent())
                         {
                             auto sumCost_ = objective_->combineCosts(vertex->getCostToComeFromGoal(),vertex->getCostToComeFromStart());
                             //cerr <<  sumCost_<< "--------- 1, "<< C_best << " " << vertex->hasForwardParent() << endl;
                             if(objective_->isCostBetterThan(sumCost_,C_best))
                             {
                                overBound_ = objective_->isCostLargerThan(vertex->getCostToComeFromStart(), vertex->getCostToComeFromGoal()) ? true : false; 
                                if(overBound_)
                                {
                                   meetValue = objective_->combineCosts(vertex->getCostToComeFromStart(), vertex->getCostToComeFromStart());
                                }
                                betterSolution(vertex,sumCost_,costToCome,costToGo);
                             }
                         }
                         updateCostToGo(vertex,costToCome, costToGo);   
        }

        void BTITstar::updateReverseCost(const std::shared_ptr<btitstar::Vertex> &vertex, ompl::base::Cost costToCome, ompl::base::Cost &costToGo)
        {
                         vertex->resetReverseExpanded();
                         if(vertex->hasForwardParent())
                         {
                             auto sumCost_ = objective_->combineCosts(vertex->getCostToComeFromGoal(),vertex->getCostToComeFromStart());
                             //cerr <<  sumCost_<< "--------- 2, " << C_best <<endl;
                             if(objective_->isCostBetterThan(sumCost_,C_best))
                             {
                                 overBound_ = objective_->isCostLargerThan(vertex->getCostToComeFromGoal(),vertex->getCostToComeFromStart()) ? true : false;
                                 if(overBound_)
                                 {
                                   meetValue = objective_->combineCosts(vertex->getCostToComeFromGoal(), vertex->getCostToComeFromGoal());
                                 } 
                                 betterSolution(vertex,sumCost_,costToCome,costToGo);
                             } 
                         }
                         updateCostToGo(vertex,costToCome,costToGo);
        }
        

        void BTITstar::printReverseStatement(const std::shared_ptr<btitstar::Vertex> &vertex)
        {
                  if(vertex->isGoal())
                  {
                     cerr << "vertex->   " << vertex->getId() << " is expanding in reverse preliminary tree. " << "  " << vertex->getCostToComeFromGoal() << "  " << vertex->getLowerCostBoundToStart() << endl;
                  }
                  else
                  {
                    ompl::base::Cost ecost(2.9); 
                    if(objective_->isCostBetterThan(ecost,solutionCost_))
                    cerr << "vertex->   " << vertex->getId() << " is expanding in reverse preliminary tree. " << vertex->getReverseParent()->getId() << "  " << vertex->getCostToComeFromGoal() << "  " << f_minb << " " << vertex->hasForwardParent()  << endl;
                  }      
        }  

        void BTITstar::ReverseSearch(const std::shared_ptr<btitstar::Vertex> &vertex) 
        {    
            //printReverseStatement(vertex);
            for (const auto &child : vertex->getReverseChildren())
            {       
                    if(findASolution_)
                    {
                       child->setPostSoluState();
                    } 
                    
                    if(vertex->hasForwardParent() && child->getId() == vertex->getForwardParent()->getId())
                       continue;
                    auto edgeCost = child->getReverseEdgeCost();//objective_->motionCostHeuristic(child->getState(), vertex->getState())
                    auto gValue = objective_->combineCosts(vertex->getCostToComeFromGoal(),edgeCost);
                    
                    auto hValue = child->getLowerCostBoundToStart();  
                         child->setCostToComeFromGoal(gValue);                                              
                         gValue = child->getCostToComeFromGoal();
                         updateReverseCost(child,gValue,hValue);
                         if(findASolution_)
                         {
                             ompl::base::Cost est_fValue = objective_->combineCosts(gValue,hValue);
                             if(objective_->isCostBetterThan(est_fValue,loc_fmin))
                             {
                                 loc_fmin = est_fValue; 
                             }
                             est_fValue = (overBound_ ? est_fValue: objective_->combineCosts(gValue,gValue));
                             betterReverse_ = couldRefineSolution(est_fValue, child->pivotalState());
                         }

                    //cerr << " old child in reverse tree " << child->getId() << " g-value: " << gValue << ", f-value: " << objective_->combineCosts(gValue,hValue) << endl;
                    if(!child->hasForwardParent() && objective_->isCostLargerThan(objective_->combineCosts(gValue,gValue),solutionCost_))
                    {      continue;   }
                    insertOrUpdateInReverseQueue(child,gValue,hValue);  
            }
            
            for (const auto &neighbor : graph_.getNeighbors(vertex))
            {  
                if(neighbor->getId() == vertex->getId())
                {
                     continue;
                }
                
                if (!neighbor->reverseExpanded())
                { 
                    if(vertex->hasForwardParent() && neighbor->getId() == vertex->getForwardParent()->getId())
                    {   
                       continue;
                    } 
                    if(neighbor->hasReverseParent() && neighbor->getReverseParent()->getId() == vertex->getId())
                    {
                        continue;
                    }
                    if (neighbor->isBlacklistedAsChild(vertex) || vertex->isBlacklistedAsChild(neighbor))
                    {
                       continue;
                    }
                    
                    auto gValue = (neighbor->hasReverseParent() ) ? neighbor->getCostToComeFromGoal() :objective_->infiniteCost();
                    //double arrTime_ = estimatedTimeDDI(neighbor->getState(),  vertex->getState());
                    //ompl::base::Cost edgeCost = estimatedCostDDI(neighbor->getState(),  vertex->getState(), arrTime_);
                    
                    //auto edgeCost = objective_->motionCostHeuristic(neighbor->getState(), vertex->getState());
                    //double arrTime_ = 0;
                    double arrTime_ = estimatedTimeMGLQ(neighbor->getState(),vertex->getState());
                    auto edgeCost = lqEdgeCostMGLQ(neighbor->getState(),vertex->getState(),arrTime_); //auto edgeCost = TrueEdgeCost(neighbor->getState(),vertex->getState());//
                    auto est_gValue = objective_->combineCosts(vertex->getCostToComeFromGoal(), edgeCost);   
                    //cerr << "????? going here " << neighbor->getId() << endl;

                    if (objective_->isCostBetterThan(est_gValue, gValue) && !neighbor->isStart())
                    {
                         if(!isValid(make_pair(neighbor,vertex),arrTime_)) 
                         {  
                                continue;
                         }
                         
                         neighbor->setCostToComeFromGoal(est_gValue); 
                         neighbor->setReverseParent(vertex,edgeCost,arrTime_);
                         vertex->addToReverseChildren(neighbor);
                         auto hValue = neighbor->getLowerCostBoundToStart();
                         bool onForwardSearch = neighbor->getForwardQueuePointer() || neighbor->hasForwardParent();   
                         if(onForwardSearch)
                         { 
                            keyEdgePair fEdge = make_pair(neighbor->getForwardParent(),neighbor);
                            auto sumCost = objective_->combineCosts(est_gValue,neighbor->getCostToComeFromStart());
                           //cerr << "\t suppose to be met {" << vertex->getId() << ", " << neighbor->getId() << "}." << " R " << Totalcost << " " << C_best << " " << est_gValue << " " <<neighbor->getCostToComeFromStart() << endl;  
                            //cerr <<  sumCost<< "--------- 4, " << C_best << endl;
                            if(objective_->isCostBetterThan(sumCost,C_best) && isValid(fEdge,neighbor->getForwardEdgeTime()))//
                            {
                                     overBound_ = objective_->isCostLargerThan(est_gValue,vertex->getCostToComeFromStart()) ? true : false; 
                                     if(overBound_)
                                     {
                                        meetValue = objective_->combineCosts(est_gValue, est_gValue);
                                     } 
                                     betterSolution(neighbor,sumCost,est_gValue,hValue);
                                     //cerr << "\t the new neighbor in both forward reverse is:  " << neighbor->getId()  << " f-value: " << objective_->combineCosts(est_gValue,hValue) << " g-value: " << est_gValue << endl;
                                     if(firstIntersection_)
                                     {  break; } 
                                     
                                     ompl::base::Cost est_fValue = objective_->combineCosts(est_gValue, hValue);
                                     
                                     if(objective_->isCostBetterThan(est_fValue,loc_fmin))
                                     {
                                        loc_fmin = est_fValue; 
                                     }
                                     
                                     insertOrUpdateInReverseQueue(neighbor,est_gValue,hValue);
                                     continue;
                            }                           
                         } 
                         updateCostToGo(neighbor,est_gValue, hValue);
                         ompl::base::Cost est_fValue = objective_->combineCosts(est_gValue, hValue);
                         if(findASolution_)
                         {    
                              neighbor->setPostSoluState();
                              if(objective_->isCostBetterThan(est_fValue,loc_fmin))
                              {
                                 loc_fmin = est_fValue; 
                              }
                              est_fValue = (overBound_ ? est_fValue: objective_->combineCosts(est_gValue,est_gValue));
                              betterReverse_ = couldRefineSolution(est_fValue, neighbor->pivotalState());
                         }  
                         if(objective_->isCostBetterThan(est_fValue, solutionCost_))  
                            insertOrUpdateInReverseQueue(neighbor,est_gValue,hValue); 
                         //cerr << "\t \t the new neighbor in reverse is:  " << neighbor->getId()  << " f-value: " << objective_->combineCosts(est_gValue,hValue) << " " << " g-value: " << est_gValue << endl;
                    } 
                }
            }
            

            if(vertex->hasForwardParent())
            {
                  auto forwardParent = vertex->getForwardParent();
                  auto edgeCost = vertex->getForwardEdgeCost(); //auto edgeCost = objective_->motionCostHeuristic(forwardParent->getState(), vertex->getState());
                  auto currentValue = objective_->combineCosts(vertex->getCostToComeFromGoal(),edgeCost);
                  auto hValue = forwardParent->getLowerCostBoundToStart();
                  forwardParent->setCostToComeFromGoal(currentValue); 
                  if(objective_->isCostBetterThan(currentValue,forwardParent->getCostToComeFromGoal()))
                  {
                         double arriveTime_ = vertex->getForwardEdgeTime(); 
                         vertex->resetForwardParent();
                         forwardParent->setReverseParent(vertex,edgeCost,arriveTime_);
                         vertex->addToReverseChildren(forwardParent);
                         updateReverseCost(forwardParent,forwardParent->getCostToComeFromGoal(),hValue);                  
                         insertOrUpdateInReverseQueue(forwardParent,forwardParent->getCostToComeFromGoal(),hValue); 
                  }
            }
        }
        
                 
        bool BTITstar::isVertexBetter(const btitstar::KeyVertexPair &lhs, const btitstar::KeyVertexPair &rhs) const
        {
                // it's a regular comparison of the keys.
                return std::lexicographical_compare(
                    lhs.first.cbegin(), lhs.first.cend(), rhs.first.cbegin(), rhs.first.cend(),
                    [this](const auto &a, const auto &b) { return objective_->isCostBetterThan(a, b); });
        }

               
        void BTITstar::betterSolution(const std::shared_ptr<Vertex> &vertex, ompl::base::Cost meetCost, ompl::base::Cost costToCome, ompl::base::Cost &costToGo)
        {                  
                           newSolution_ = true;  
                           //cerr << "\t\t found the current best solution " << vertex->getId() << endl;
                           if(V_meet.second)
                           {  V_meet.second->resetMeetingState(); }
                           vertex->setMeetingState();
                           V_meet = make_pair(C_best= meetCost,vertex);
                           updateCostToGo(vertex,costToCome, costToGo); 
        }
        void BTITstar::updateCostToGo(const std::shared_ptr<Vertex> &vertex, ompl::base::Cost &costToCome, ompl::base::Cost &costToGo)
        {
                    if(objective_->isCostBetterThan(costToGo,costToCome))
                    {
                            costToGo =   costToCome;
                            vertex->setPivotalState();
                    }           
        }
        
        void BTITstar::insertOrUpdateInReverseQueue(const std::shared_ptr<btitstar::Vertex> &vertex, ompl::base::Cost CostToCome, ompl::base::Cost CostToGoal)
        {
          auto element = vertex->getReverseQueuePointer();
          //Update it if it is in
          if(element) {
                 element->data.first = computeEstimatedPathCosts(CostToCome,CostToGoal);
                 reverseQueue_.update(element);
          }
          else //Insert the pointer into the queue
          {
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> element(computeEstimatedPathCosts(CostToCome,CostToGoal),
                                                                                          vertex);
                // Insert the vertex into the queue, storing the corresponding pointer.
                auto reverseQueuePointer = reverseQueue_.insert(element);
                vertex->setReverseQueuePointer(reverseQueuePointer);
          }
        }  
        
        void BTITstar::insertOrUpdateInForwardQueue(const std::shared_ptr<btitstar::Vertex> &vertex, ompl::base::Cost CostToCome, ompl::base::Cost CostToGoal)
        {
          //Get the pointer to the element in the queue
          auto element = vertex->getForwardQueuePointer();
          //Update it if it is in
          if(element) {
                 element->data.first = computeEstimatedPathCosts(CostToCome,CostToGoal);
                 forwardQueue_.update(element);
          }
          else //Insert the pointer into the queue
          {
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> element(computeEstimatedPathCosts(CostToCome,CostToGoal),
                                                                                             vertex);
                // Insert the vertex into the queue, storing the corresponding pointer.
                auto forwardQueuePointer = forwardQueue_.insert(element);
                vertex->setForwardQueuePointer(forwardQueuePointer);
          }
        }
        
        
        ompl::base::Cost BTITstar::MaxValue(const ompl::base::Cost v1, const ompl::base::Cost v2) const
        {      return objective_->isCostLargerThan(v1, v2) ? v1 : v2;               }   
           
        std::shared_ptr<ompl::geometric::PathGeometric> BTITstar::getPathToVertex(const std::shared_ptr<Vertex> &vertex) const
        {
            // Create the reverse path by following the parents in forward tree to the start from the middler.
            std::vector<std::shared_ptr<Vertex>> reversePath;
            auto current = vertex;
            //cerr << "???111?" << " " << vertex->getId() << endl;
            ompl::base::Cost max_min = objective_->identityCost();
            while (!graph_.isStart(current))
            {
                //const ompl::base::Cost C_ = objective_->combineCosts(current->getLowerCostBoundToStart(),current->getLowerCostBoundToGoal());
                //max_min = MaxValue(C_,max_min); 
                reversePath.emplace_back(current);
                current = current->getForwardParent();
                //cerr << current->getId() << endl;
            }
            reversePath.emplace_back(current);

            // Reverse the reverse path to get the forward path.
            auto path = std::make_shared<ompl::geometric::PathGeometric>(Planner::si_);
            for (const auto &vertex_ : boost::adaptors::reverse(reversePath))
            {   
                //cerr << vertex_->getId() << endl;
                //spaceInformation_->printState(vertex_->getState());
                path->append(vertex_->getState());
            }
            reversePath.clear();
            // Trace back the forward path by following the parents in reverse tree to goal from the middler
            current = vertex; 
            while (!graph_.isGoal(current))
            {
                //const ompl::base::Cost C_ = objective_->combineCosts(current->getLowerCostBoundToStart(),current->getLowerCostBoundToGoal());
                //max_min = MaxValue(C_,max_min); 
                reversePath.emplace_back(current);
                current = current->getReverseParent();
            }
            reversePath.emplace_back(current);
            for (const auto &vertex_ : reversePath)
            { 
                if(vertex_->getId() != vertex->getId())
                {
                  //cerr << vertex_->getId() << endl;
                  //spaceInformation_->printState(vertex_->getState());
                  path->append(vertex->getState());
                }   
            }
            //ompl::base::Cost path1_ = objective_->combineCosts(vertex->getLowerCostBoundToStart(),vertex->getLowerCostBoundToGoal());
            //ompl::base::Cost path2_ = objective_->combineCosts(vertex->getLowerCostBoundToGoal(),vertex->getLowerCostBoundToStart());
            //samplingBound_ = max_min;
            //cerr << path1_ << " ---- " << path2_ << " " << samplingBound_ << " " << max_min << endl;
            return path;
        }

        std::array<ompl::base::Cost, 2u> BTITstar::computeEstimatedPathCosts(ompl::base::Cost CostToStart, ompl::base::Cost CostToGoal) const
        {
            // f = g(x) + h(x); g(x) = g(parent(x))+c(parent(x),x)
            return {objective_->combineCosts(CostToStart,CostToGoal),CostToStart};
        }
        
        bool BTITstar::selectExpansion(bool &forwardDir_)
        {
            auto forwardVertex_ = forwardQueue_.top()->data.second;
            auto backwardVertex_ = reverseQueue_.top()->data.second;
                 f_minf = forwardQueue_.top()->data.first[0u];
                 f_minb = reverseQueue_.top()->data.first[0u];
            
                 
            if(forwardVertex_->forwardExpanded())
            {
                    forwardQueue_.pop();
                    return false;  
            }
            if(backwardVertex_->reverseExpanded())
            {
                    reverseQueue_.pop();
                    return false;
            }
            // If the minimum priority is from forward search
            if(objective_->isCostBetterThan(f_minf, f_minb))
            {
                   BestVertex_ = forwardQueue_.top()->data.second;
                   forwardQueue_.pop();
                   BestVertex_->resetForwardQueuePointer();
                   f_min = f_minf;
                   f_max = f_minb;
                   currGValue_ = BestVertex_->getCostToComeFromStart();
                   currHValue_ = BestVertex_->getLowerCostBoundToGoal();
                   forwardDir_ = true;
            } 
            else 
            {   
                   BestVertex_ = reverseQueue_.top()->data.second;
                   reverseQueue_.pop();
                   BestVertex_->resetReverseQueuePointer(); 
                   f_min = f_minb;
                   f_max = f_minf;
                   currGValue_ = BestVertex_->getCostToComeFromGoal();
                   currHValue_ = BestVertex_->getLowerCostBoundToStart();
            }
            return true;
        }
        
        bool BTITstar::isValid(const keyEdgePair &edge, double arriveTime_)
        {
        
            auto parent = edge.first;
            auto child = edge.second;
            //spaceInformation_->printState(parent->getState());
            //spaceInformation_->printState(child->getState()); 
            // Check if the edge is whitelisted.
            if (parent->isWhitelistedAsChild(child))
            {
                return true;
            }

            // If the edge is blacklisted.
            if (child->isBlacklistedAsChild(parent))
            {
                return false;
            }

            // Get the segment count for the full resolution.
            const std::size_t fullSegmentCount = 200;//space_->validSegmentCount(parent->getState(), child->getState())

            // The segment count is the number of checks on this level plus 1, capped by the full resolution segment
            // count.
            const auto segmentCount = 200;//std::min(numChecks + 1u, fullSegmentCount)
           
            // Store the current check number.
            std::size_t currentCheck = 1u;

            // Get the number of checks already performed on this edge.
            const std::size_t performedChecks = 0u;//child->getIncomingCollisionCheckResolution(parent->getId())
            // Initialize the queue of positions to be tested.
            std::queue<std::pair<std::size_t, std::size_t>> indices;
            indices.emplace(1u, 200);
            
            // Test states while there are states to be tested.
            while (!indices.empty())
            {
                // Get the current segment.
                const auto current = indices.front();

                // Get the midpoint of the segment.
                auto mid = (current.first + current.second) / 2;
                //cerr << current.first << "  " << current.second << endl;
                //cerr << currentCheck << " " << performedChecks << endl;
                // Only do the detection if we haven't tested this state on a previous level.
                if (currentCheck > performedChecks)
                {
                    double t = (static_cast<double>(mid) / static_cast<double>(segmentCount))*arriveTime_;
                    interpolateMGLQ(t, arriveTime_, parent->getState(), child->getState(), detectionState_, false);
                    //space_->interpolate(parent->getState(), child->getState(),static_cast<double>(mid) / static_cast<double>(segmentCount), detectionState_);
                    //interpolateDDI(t, arriveTime_, parent->getState(), child->getState(), detectionState_, false);
                    //interpolateGL(t, detectionState_, false);
                    //cerr << "midding------- > " << mid << " " << currentCheck << " " << segmentCount << " "<< ((static_cast<double>(mid) / static_cast<double>(segmentCount))*3.323140348909138)<< endl;
                    //spaceInformation_->printState(detectionState_);
                    if (!spaceInformation_->isValid(detectionState_))
                    {
                        // Blacklist the edge.
                        parent->blacklistAsChild(child);
                        child->blacklistAsChild(parent);
                        // Register it with the graph.
                        return false;
                    }
                }

                // Remove the current segment from the queue.
                indices.pop();

                // Create the first and second half of the split segment if necessary.
                if (current.first < mid)
                {
                    indices.emplace(current.first, mid - 1u);
                }

                if (current.second > mid)
                {
                    indices.emplace(mid + 1u, current.second);
                }

                // Increase the current check number.
                ++currentCheck;
            }

            // Remember at what resolution this edge was already checked. We're assuming that the number of collision
            // checks is symmetric for each edge.

            // Whitelist this edge if it was checked at full resolution.
            if (segmentCount == fullSegmentCount)
            {   
                //++numCollisionCheckedEdges_;
                parent->whitelistAsChild(child);
                child->whitelistAsChild(parent);
            }
            return true;
        }  

    }  // namespace geometric
}  
