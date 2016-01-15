/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan, Javier V Gomez */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_DRRTSTARFN_
#define OMPL_GEOMETRIC_PLANNERS_RRT_DRRTSTARFN_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <limits>
#include <vector>
#include <list>
#include <utility>

namespace ompl {

namespace geometric {

    // based on RRTstar.h

    /** \brief Optimal Rapidly-exploring Random Trees */
    class DRRTstarFN : public base::Planner {
        friend class boost::serialization::access;

    public:
        DRRTstarFN(const base::SpaceInformationPtr& si);

        virtual ~DRRTstarFN();

        virtual void getPlannerData(base::PlannerData& data) const;

        virtual base::PlannerStatus solve(const base::PlannerTerminationCondition& ptc);

        virtual void clear();

        /** \brief Set the goal bias

          In the process of randomly selecting states in
          the state space to attempt to go towards, the
          algorithm may in fact choose the actual goal state, if
          it knows it, with some probability. This probability
          is a real number between 0.0 and 1.0; its value should
          usually be around 0.05 and should not be too large. It
          is probably a good idea to use the default value. */
        void setGoalBias(double goalBias)
        {
            goalBias_ = goalBias;
        }

        /** \brief Get the goal bias the planner is using */
        double getGoalBias() const
        {
            return goalBias_;
        }

        /** \brief fraction of the time in the dynamic part of
         * the motion planning will be biased with the following variable */
        void setOrphanedBias(double orphanedBias)
        {
            orphanedBias_ = orphanedBias;
        }
        /** \brief  get the biasing factor for orphaned nodes*/
        double getOrphanedBias()
        {
            return orphanedBias_;
        }

        /** \brief Set the range the planner is supposed to use.

          This parameter greatly influences the runtime of the
          algorithm. It represents the maximum length of a
          motion to be added in the tree of motions. */
        void setRange(double distance)
        {
            maxDistance_ = distance;
        }

        /** \brief Get the range the planner is using */
        double getRange() const
        {
            return maxDistance_;
        }

        /** \brief Set a different nearest neighbors datastructure */
        template <template <typename T> class NN>
        void setNearestNeighbors()
        {
            nn_.reset(new NN<Motion*>());
        }
        template <typename T>
        void setNN(NearestNeighbors<T*> nearestNeighbors)
        {
            nn_.reset(nearestNeighbors);
        }

        /** \brief Option that delays collision checking procedures.
          When it is enabled, all neighbors are sorted by cost. The
          planner then goes through this list, starting with the lowest
          cost, checking for collisions in order to find a parent. The planner
          stops iterating through the list when a collision free parent is found.
          This prevents the planner from collsion checking each neighbor, reducing
          computation time in scenarios where collision checking procedures are expensive.*/
        void setDelayCC(bool delayCC)
        {
            delayCC_ = delayCC;
        }

        /** \brief Get the state of the delayed collision checking option */
        bool getDelayCC() const
        {
            return delayCC_;
        }

        /** \brief Set the percentage threshold (between 0 and 1) for pruning the tree. If the new tree has removed
          at least this percentage of states, the tree will be finally pruned. */
        void setPruneStatesImprovementThreshold(const double pp)
        {
            pruneStatesThreshold_ = pp;
        }

        /** \brief Get the current prune states percentage threshold parameter. */
        double getPruneStatesImprovementThreshold() const
        {
            return pruneStatesThreshold_;
        }

        virtual void setup();

        ///////////////////////////////////////
        // Planner progress property functions
        std::string getIterationCount() const
        {
            return boost::lexical_cast<std::string>(iterations_);
        }
        std::string getBestCost() const
        {
            return boost::lexical_cast<std::string>(bestCost_);
        }

        void setMaxNodes(unsigned int nodesNum)
        {
            maxNodes_ = nodesNum;
        }

        unsigned int getMaxNodes() const
        {
            return maxNodes_;
        }

        void setLocalPlanning(bool set)
        {
            localPlanning_ = set;
        }
        /** \brief Set sampling radius around interim state */
        bool isLocalPlanning()
        {
            return localPlanning_;
        }

        /** \brief Set the interim state */
        void setInterimState(base::State* state)
        {
            interimState_ = state;
        }

        /** \brief Set sampling radius around the interim state */
        void setSampleRadius(double r)
        {
            sampleRadius_ = r;
        }

        /** \brief Remove the states from the tree */
        int removeInvalidNodes();

        /** \brief here the algorithm will find invalid nodes and mark/remove them from the tree */
        void markForRemoval();

        /** \brief remove orphaned nodes from the tree */

        void removeOrphaned();

        /** \brief here the algorithm will try to connect orphaned nodes to the rest of the tree */
        void stepTwo();

        /** \brief Save the state of the tree */

        void restoreTree(const char* filename);

        /** \brief Load the state of the tree */

        //void loadTree(const char *filename);

        enum NodeType : char { NORMAL = 0,
            ORPHANED = 1,
            NEW_DYNAMIC = 2,
            REMOVED = 3 };

    protected:
        /** \brief Representation of a motion */
        class Motion {
        public:
            /** \brief Constructor that allocates memory for the state. This constructor automatically allocates memory for \e state, \e cost, and \e incCost */
            Motion(const base::SpaceInformationPtr& si)
                : state(si->allocState())
                , parent(NULL)
                , nodeType(NORMAL)
            {
            }

            ~Motion()
            {
            }

            /** \brief The state contained by the motion */
            base::State* state;

            /** \brief The parent motion in the exploration tree */
            Motion* parent;

            /** \brief The cost up to this motion */
            base::Cost cost;

            /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance computations in the updateChildCosts() method) */
            base::Cost incCost;

            /** \brief The set of motions descending from the current motion */
            std::vector<Motion*> children;

            /** \brief removed */
            NodeType nodeType;
        };

        /** \brief Free the memory allocated by this planner */
        void freeMemory();

        // For sorting a list of costs and getting only their sorted indices
        struct CostIndexCompare {
            CostIndexCompare(const std::vector<base::Cost>& costs,
                const base::OptimizationObjective& opt)
                : costs_(costs)
                , opt_(opt)
            {
            }
            bool operator()(unsigned i, unsigned j)
            {
                return opt_.isCostBetterThan(costs_[i], costs_[j]);
            }
            const std::vector<base::Cost>& costs_;
            const base::OptimizationObjective& opt_;
        };

        /** \brief Compute distance between motions (actually distance between contained states) */
        double distanceFunction(const Motion* a, const Motion* b) const
        {
            return si_->distance(a->state, b->state);
        }

        /** \brief Removes the given motion from the parent's child list */
        void removeFromParent(Motion* m);

        /** \brief Updates the cost of the children of this node if the cost up to this node has changed */
        void updateChildCosts(Motion* m);

        /** \brief Prunes all those states which estimated total cost is higher than pruneTreeCost.
          Returns the number of motions pruned. Depends on the parameter set by setPruneStatesImprovementThreshold() */
        //int pruneTree(const base::Cost pruneTreeCost);

        /** \brief Deletes (frees memory) the motion and its children motions. */
        void deleteBranch(Motion* motion);

        /** \brief Deletes (frees memory) the motion that was invalidated
          by a dynamic obstacle and generating a list of orphaned branches. */
        bool huntKids(Motion* motion, std::vector<Motion*>& orph);

        void verifyTree();

        /** \brief Marks the branch as orphaned */
        void markOrphaned(Motion* m);

        void markNormal(Motion* m);

        // TODO write an explanation
        bool traverseTree(const unsigned int n, const ompl::base::PlannerData& pdat);

        /** \brief Computes the Cost To Go heuristically as the cost to come from start to motion plus
          the cost to go from motion to goal. If \e shortest is true, the estimated cost to come
          start-motion is given. Otherwise, this cost to come is the current motion cost. */
        base::Cost costToGo(const Motion* motion, const bool shortest = true) const;

        /** \brief State sampler */
        base::StateSamplerPtr sampler_;

        /** \brief A nearest-neighbors datastructure containing the tree of motions */
        boost::shared_ptr<NearestNeighbors<Motion*> > nn_;

        /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
        double goalBias_;

        /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
        double orphanedBias_;

        /** \brief The maximum length of a motion to be added to a tree */
        double maxDistance_;

        /** \brief The random number generator */
        RNG rng_;

        /** \brief Option to delay and reduce collision checking within iterations */
        bool delayCC_;

        /** \brief Objective we're optimizing */
        base::OptimizationObjectivePtr opt_;

        /** \brief The most recent goal motion.  Used for PlannerData computation */
        Motion* lastGoalMotion_;

        /** \brief A list of states in the tree that satisfy the goal condition */
        std::vector<Motion*> goalMotions_;

        /** \brief If this value is set to true, tree pruning will be enabled. */
        //bool                                           prune_;

        /** \brief The tree is only pruned is the percentage of states to prune is above this threshold (between 0 and 1). */
        double pruneStatesThreshold_;

        struct PruneScratchSpace {
            std::vector<Motion*> newTree, toBePruned, candidates;
        } pruneScratchSpace_;

        /** \brief Stores the Motion containing the last added initial start state. */
        Motion* startMotion_;

        //////////////////////////////
        // Planner progress properties
        /** \brief Number of iterations the algorithm performed */
        unsigned int iterations_;
        /** \brief Best cost found so far by algorithm */
        base::Cost bestCost_;

        unsigned int maxNodes_;

        bool localPlanning_;

        base::State* interimState_;
        double sampleRadius_;
        std::vector<Motion*> orphanedNodes_;
    };
}
}

#endif
