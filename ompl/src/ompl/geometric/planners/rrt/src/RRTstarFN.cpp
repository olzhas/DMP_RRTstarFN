/*********************************************************************f
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

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan
 * Sucan, Javier V Gomez */

#include "ompl/geometric/planners/rrt/RRTstarFN.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"

#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <limits>

constexpr std::size_t kDefaultMaxNodes = 30000;

ompl::geometric::RRTstarFN::RRTstarFN(const base::SpaceInformationPtr &si)
    : base::Planner(si, "RRTstarFN"),
      goalBias_(0.05),
      maxDistance_(0.0),
      delayCC_(true),
      lastGoalMotion_(NULL),
      iterations_(0),
      bestCost_(std::numeric_limits<double>::quiet_NaN()),
      maxNodes_(kDefaultMaxNodes) {
  specs_.approximateSolutions = true;
  specs_.optimizingPaths = true;
  specs_.canReportIntermediateSolutions = true;

  Planner::declareParam<double>("range", this, &RRTstarFN::setRange,
                                &RRTstarFN::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &RRTstarFN::setGoalBias,
                                &RRTstarFN::getGoalBias, "0.:.05:1.");
  Planner::declareParam<bool>("delay_collision_checking", this,
                              &RRTstarFN::setDelayCC, &RRTstarFN::getDelayCC,
                              "0,1");
  Planner::declareParam<unsigned int>("maxNodes", this, &RRTstarFN::setMaxNodes,
                                      &RRTstarFN::getMaxNodes, "100:1:100000");

  addPlannerProgressProperty("iterations INTEGER",
                             std::bind(&RRTstarFN::getIterationCount, this));
  addPlannerProgressProperty("best cost REAL",
                             std::bind(&RRTstarFN::getBestCost, this));
}

ompl::geometric::RRTstarFN::~RRTstarFN() { freeMemory(); }

void ompl::geometric::RRTstarFN::setup() {
  Planner::setup();
  tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);
  if (!si_->getStateSpace()->hasSymmetricDistance() ||
      !si_->getStateSpace()->hasSymmetricInterpolate()) {
    OMPL_WARN(
        "%s requires a state space with symmetric distance and symmetric "
        "interpolation.",
        getName().c_str());
  }

  if (!nn_)
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
  nn_->setDistanceFunction(std::bind(&RRTstarFN::distanceFunction, this,
                                     std::placeholders::_1,
                                     std::placeholders::_2));

  // Setup optimization objective
  //
  // If no optimization objective was specified, then default to
  // optimizing path length as computed by the distance() function
  // in the state space.
  if (pdef_) {
    if (pdef_->hasOptimizationObjective())
      opt_ = pdef_->getOptimizationObjective();
    else {
      OMPL_INFORM(
          "%s: No optimization objective specified. Defaulting to optimizing "
          "path length for the allowed planning time.",
          getName().c_str());
      opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }
  } else {
    OMPL_INFORM(
        "%s: problem definition is not set, deferring setup completion...",
        getName().c_str());
    setup_ = false;
  }

  rng_.setLocalSeed(32);
}

void ompl::geometric::RRTstarFN::clear() {
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if (nn_) nn_->clear();

  lastGoalMotion_ = NULL;
  goalMotions_.clear();

  iterations_ = 0;
  bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

ompl::base::PlannerStatus ompl::geometric::RRTstarFN::solve(
    const base::PlannerTerminationCondition &ptc) {
  checkValidity();
  base::Goal *goal = pdef_->getGoal().get();
  base::GoalSampleableRegion *goal_s =
      dynamic_cast<base::GoalSampleableRegion *>(goal);

  bool symCost = opt_->isSymmetric();

  while (const base::State *st = pis_.nextStart()) {
    Motion *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    motion->cost = opt_->identityCost();
    nn_->add(motion);
    startMotion_ = motion;
  }

  if (nn_->size() == 0) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  if (!sampler_) sampler_ = si_->allocStateSampler();

  OMPL_INFORM("%s: Starting planning with %u states already in datastructure",
              getName().c_str(), nn_->size());

  const base::ReportIntermediateSolutionFn intermediateSolutionCallback =
      pdef_->getIntermediateSolutionCallback();

  Motion *solution = lastGoalMotion_;

  // \TODO Make this variable unnecessary, or at least have it
  // persist across solve runs
  base::Cost bestCost = opt_->infiniteCost();

  bestCost_ = opt_->infiniteCost();

  Motion *approximation = NULL;
  double approximatedist = std::numeric_limits<double>::infinity();
  bool sufficientlyShort = false;

  Motion *rmotion = new Motion(si_);
  base::State *rstate = rmotion->state;
  base::State *xstate = si_->allocState();

  // e+e/d.  K-nearest RRT*
  double k_rrg = boost::math::constants::e<double>() +
                 (boost::math::constants::e<double>() /
                  (double)si_->getStateSpace()->getDimension());

  std::vector<Motion *> nbh;

  std::vector<base::Cost> costs;
  std::vector<base::Cost> incCosts;
  std::vector<std::size_t> sortedCostIndices;

  std::vector<int> valid;
  unsigned int rewireTest = 0;
  unsigned int statesGenerated = 0;

  if (solution)
    OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f",
                getName().c_str(), solution->cost.value());
  OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
              (unsigned int)std::ceil(k_rrg * log((double)(nn_->size() + 1))));

  // our functor for sorting nearest neighbors
  CostIndexCompare compareFn(costs, *opt_);

  while (ptc == false) {
    iterations_++;

    // sample random state (with goal biasing)
    // Goal samples are only sampled until maxSampleCount() goals are in the
    // tree, to prohibit duplicate goal states.
    if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() &&
        rng_.uniform01() < goalBias_ && goal_s->canSample())
      goal_s->sampleGoal(rstate);
    else
      sampler_->sampleUniform(rstate);

    // find closest state in the tree
    Motion *nmotion = nn_->nearest(rmotion);

    if (intermediateSolutionCallback &&
        si_->equalStates(nmotion->state, rstate))
      continue;

    base::State *dstate = rstate;

    // find state to add to the tree
    double d = si_->distance(nmotion->state, rstate);
    if (d > maxDistance_) {
      si_->getStateSpace()->interpolate(nmotion->state, rstate,
                                        maxDistance_ / d, xstate);
      dstate = xstate;
    }

    // Check if the motion between the nearest state and the state to add is
    // valid
    if (si_->checkMotion(nmotion->state,
                         dstate))  // && maxNodes_ > statesGenerated)
    {
      // create a motion
      Motion *motion = new Motion(si_);
      si_->copyState(motion->state, dstate);
      motion->parent = nmotion;
      motion->incCost = opt_->motionCost(nmotion->state, motion->state);
      motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

      // Find nearby neighbors of the new motion - k-nearest RRT*
      unsigned int k = std::ceil(k_rrg * log((double)(nn_->size() + 1)));
      nn_->nearestK(motion, k, nbh);
      rewireTest += nbh.size();
      statesGenerated++;

      // cache for distance computations
      //
      // Our cost caches only increase in size, so they're only
      // resized if they can't fit the current neighborhood
      if (costs.size() < nbh.size()) {
        costs.resize(nbh.size());
        incCosts.resize(nbh.size());
        sortedCostIndices.resize(nbh.size());
      }

      // cache for motion validity (only useful in a symmetric space)
      //
      // Our validity caches only increase in size, so they're
      // only resized if they can't fit the current neighborhood
      if (valid.size() < nbh.size()) valid.resize(nbh.size());
      std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

      // Finding the nearest neighbor to connect to
      // By default, neighborhood states are sorted by cost, and collision
      // checking
      // is performed in increasing order of cost
      if (delayCC_) {
        // calculate all costs and distances
        for (std::size_t i = 0; i < nbh.size(); ++i) {
          incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
          costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
        }

        // sort the nodes
        //
        // we're using index-value pairs so that we can get at
        // original, unsorted indices
        for (std::size_t i = 0; i < nbh.size(); ++i) sortedCostIndices[i] = i;
        std::sort(sortedCostIndices.begin(),
                  sortedCostIndices.begin() + nbh.size(), compareFn);

        // collision check until a valid motion is found
        //
        // ASYMMETRIC CASE: it's possible that none of these
        // neighbors are valid. This is fine, because motion
        // already has a connection to the tree through
        // nmotion (with populated cost fields!).
        for (std::vector<std::size_t>::const_iterator i =
                 sortedCostIndices.begin();
             i != sortedCostIndices.begin() + nbh.size(); ++i) {
          if (nbh[*i] == nmotion ||
              si_->checkMotion(nbh[*i]->state, motion->state)) {
            motion->incCost = incCosts[*i];
            motion->cost = costs[*i];
            motion->parent = nbh[*i];
            valid[*i] = 1;
            break;
          } else
            valid[*i] = -1;
        }
      } else  // if not delayCC
      {
        motion->incCost = opt_->motionCost(nmotion->state, motion->state);
        motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
        // find which one we connect the new state to
        for (std::size_t i = 0; i < nbh.size(); ++i) {
          if (nbh[i] != nmotion) {
            incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
            costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
            if (opt_->isCostBetterThan(costs[i], motion->cost)) {
              if (si_->checkMotion(nbh[i]->state, motion->state)) {
                motion->incCost = incCosts[i];
                motion->cost = costs[i];
                motion->parent = nbh[i];
                valid[i] = 1;
              } else
                valid[i] = -1;
            }
          } else {
            incCosts[i] = motion->incCost;
            costs[i] = motion->cost;
            valid[i] = 1;
          }
        }
      }

      // boost::shared_ptr< NearestNeighbors<Motion*> > backup_nn_ =
      // boost::shared_ptr(nn_);
      // if(maxNodes_ < statesGenerated)

      // add motion to the tree
      nn_->add(motion);
      motion->parent->children.push_back(motion);

      bool checkForSolution = false;
      for (std::size_t i = 0; i < nbh.size(); ++i) {
        if (nbh[i] != motion->parent) {
          base::Cost nbhIncCost;
          if (symCost)
            nbhIncCost = incCosts[i];
          else
            nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
          base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
          if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost)) {
            bool motionValid;
            if (valid[i] == 0)
              motionValid = si_->checkMotion(motion->state, nbh[i]->state);
            else
              motionValid = (valid[i] == 1);

            if (motionValid) {
              // Remove this node from its parent list
              removeFromParent(nbh[i]);

              // Add this node to the new parent
              nbh[i]->parent = motion;
              nbh[i]->incCost = nbhIncCost;
              nbh[i]->cost = nbhNewCost;
              nbh[i]->parent->children.push_back(nbh[i]);

              // Update the costs of the node's children
              updateChildCosts(nbh[i]);

              checkForSolution = true;
            }
          }
        }
      }

      // Add the new motion to the goalMotion_ list, if it satisfies the goal
      double distanceFromGoal;

      if (goal->isSatisfied(motion->state, &distanceFromGoal)) {
        goalMotions_.push_back(motion);
        checkForSolution = true;
      }

      // Checking for solution or iterative improvement
      if (checkForSolution) {
        bool updatedSolution = false;
        for (size_t i = 0; i < goalMotions_.size(); ++i) {
          if (opt_->isCostBetterThan(goalMotions_[i]->cost, bestCost)) {
            bestCost = goalMotions_[i]->cost;
            bestCost_ = bestCost;
            updatedSolution = true;
          }

          sufficientlyShort = opt_->isSatisfied(goalMotions_[i]->cost);
          if (sufficientlyShort) {
            solution = goalMotions_[i];
            break;
          } else if (!solution ||
                     opt_->isCostBetterThan(goalMotions_[i]->cost,
                                            solution->cost)) {
            solution = goalMotions_[i];
            updatedSolution = true;
          }
        }

        if (updatedSolution) {
          if (intermediateSolutionCallback) {
            std::vector<const base::State *> spath;
            Motion *intermediate_solution =
                solution
                    ->parent;  // Do not include goal state to simplify code.

            do {
              spath.push_back(intermediate_solution->state);
              intermediate_solution = intermediate_solution->parent;
            } while (intermediate_solution->parent !=
                     0);  // Do not include the start state.

            intermediateSolutionCallback(this, spath, bestCost_);
          }
        }
      }

      // Checking for approximate solution (closest state found to the goal)
      if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist) {
        approximation = motion;
        approximatedist = distanceFromGoal;
      }

      if (statesGenerated > maxNodes_) {
        // OMPL_INFORM("%d > %d", statesGenerated, maxNodes_);
        std::vector<Motion *> motions;
        nn_->list(motions);
        std::vector<std::size_t> childlessNodes;
        for (std::size_t i = 0; i < motions.size(); ++i) {
          if (motions[i]->children.size() == 0) childlessNodes.push_back(i);
        }

        if (childlessNodes.size() > 0) {
          std::size_t rmNode = rng_.uniformInt(0, childlessNodes.size() - 1);
          OMPL_INFORM("childless %d", childlessNodes.size());
          std::size_t &ind = childlessNodes[rmNode];
          std::vector<Motion *> &childVec = motions[ind]->parent->children;

          auto it = std::find(childVec.begin(), childVec.end(), motions[ind]);
          if (it != childVec.end()) {
            childVec.erase(it);
          }

          bool rmResult = nn_->remove(motions[childlessNodes[rmNode]]);
          if (rmResult == false) OMPL_WARN("cannot remove the node");
        } else
          OMPL_WARN("zero childless nodes");
      }
    }

    // terminate if a sufficient solution is found
    if (solution && sufficientlyShort) break;
  }

  bool approximate = (solution == NULL);
  bool addedSolution = false;
  if (approximate)
    solution = approximation;
  else
    lastGoalMotion_ = solution;

  if (solution != NULL) {
    ptc.terminate();
    // construct the solution path
    std::vector<Motion *> mpath;
    while (solution != NULL) {
      mpath.push_back(solution);
      solution = solution->parent;
    }

    // set the solution path
    PathGeometric *geoPath = new PathGeometric(si_);
    for (int i = mpath.size() - 1; i >= 0; --i)
      geoPath->append(mpath[i]->state);

    base::PathPtr path(geoPath);
    // Add the solution path.
    base::PlannerSolution psol(path);
    psol.setPlannerName(getName());
    if (approximate) psol.setApproximate(approximatedist);
    // Does the solution satisfy the optimization objective?
    psol.setOptimized(opt_, bestCost, sufficientlyShort);
    pdef_->addSolutionPath(psol);

    addedSolution = true;
  }

  si_->freeState(xstate);
  if (rmotion->state) si_->freeState(rmotion->state);
  delete rmotion;

  OMPL_INFORM(
      "%s: Created %u new states. Checked %u rewire options. %u goal states in "
      "tree.",
      getName().c_str(), statesGenerated, rewireTest, goalMotions_.size());

  return base::PlannerStatus(addedSolution, approximate);
}

void ompl::geometric::RRTstarFN::removeFromParent(Motion *m) {
  for (std::vector<Motion *>::iterator it = m->parent->children.begin();
       it != m->parent->children.end(); ++it)
    if (*it == m) {
      m->parent->children.erase(it);
      break;
    }
}

void ompl::geometric::RRTstarFN::updateChildCosts(Motion *m) {
  for (std::size_t i = 0; i < m->children.size(); ++i) {
    m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
    updateChildCosts(m->children[i]);
  }
}

void ompl::geometric::RRTstarFN::freeMemory() {
  if (nn_) {
    std::vector<Motion *> motions;
    nn_->list(motions);
    for (std::size_t i = 0; i < motions.size(); ++i) {
      if (motions[i]->state) si_->freeState(motions[i]->state);
      delete motions[i];
    }
  }
}

void ompl::geometric::RRTstarFN::getPlannerData(base::PlannerData &data) const {
  Planner::getPlannerData(data);

  std::vector<Motion *> motions;
  if (nn_) nn_->list(motions);

  if (lastGoalMotion_)
    data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

  for (std::size_t i = 0; i < motions.size(); ++i) {
    if (motions[i]->parent == NULL)
      data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
    else
      data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                   base::PlannerDataVertex(motions[i]->state));
  }
}

ompl::base::Cost ompl::geometric::RRTstarFN::costToGo(
    const Motion *motion, const bool shortest) const {
  base::Cost costToCome;
  if (shortest)
    costToCome = opt_->motionCost(startMotion_->state, motion->state);  // h_s
  else
    costToCome = motion->cost;  // d_s

  const base::Cost costToGo =
      base::goalRegionCostToGo(motion->state, pdef_->getGoal().get());  // h_g
  return opt_->combineCosts(costToCome, costToGo);  // h_s + h_g
}
