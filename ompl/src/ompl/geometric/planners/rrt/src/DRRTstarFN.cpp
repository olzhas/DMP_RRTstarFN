#include "ompl/geometric/planners/rrt/DRRTstarFN.h"
#include "ompl/base/DelayedTerminationCondition.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerDataStorage.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"

#include <algorithm>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>

// boost
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>

constexpr std::size_t kDefaultMaxNodes{30000};

ompl::geometric::DRRTstarFN::DRRTstarFN(const base::SpaceInformationPtr& si)
    : DynamicPlanner(si, "DRRTstarFN") {}

ompl::geometric::DRRTstarFN::~DRRTstarFN() { freeMemory(); }

void ompl::geometric::DRRTstarFN::setup() {
  DynamicPlanner::setup();
  tools::SelfConfig sc(getSpaceInformation(), getName());
  sc.configurePlannerRange(maxDistance_);
  if (!si_->getStateSpace()->hasSymmetricDistance() ||
      !si_->getStateSpace()->hasSymmetricInterpolate()) {
    OMPL_WARN(
        "%s requires a state space with symmetric distance and symmetric "
        "interpolation.",
        getName().c_str());
  }

  if (!nn_) {
    // TODO implement equal sized grid to map the state space
    nn_.reset(new NearestNeighborsLinear<Motion*>());
    //
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
  }
  nn_->setDistanceFunction(std::bind(&DRRTstarFN::distanceFunction, this,
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
}

void ompl::geometric::DRRTstarFN::clear() {
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if (nn_) nn_->clear();

  lastGoalMotion_ = nullptr;
  goalMotions_.clear();

  iterations_ = 0;
  bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

ompl::base::PlannerStatus ompl::geometric::DRRTstarFN::solve(
    const base::PlannerTerminationCondition& ptc) {
  int removedNodes = 0;

  checkValidity();
  base::Goal* goal = pdef_->getGoal().get();
  base::GoalSampleableRegion* goal_s =
      dynamic_cast<base::GoalSampleableRegion*>(goal);
  // std::vector<Motion*> orphans;

  bool symCost = opt_->isSymmetric();

  // here we define a starting point in the tree
  if (!localPlanning_) {
    std::vector<Motion*> motions;
    nn_->list(motions);
    while (const base::State* st = pis_.nextStart()) {
      auto it = std::find_if(motions.begin(), motions.end(), [&](Motion* m) {
        return si_->equalStates(m->state, st);
      });
      // skip if the start state is already in the tree
      if (it == motions.end()) {
        Motion* motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->cost = opt_->identityCost();
        nn_->add(motion);
        startMotion_ = motion;
      }
    }
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

  Motion* solution = lastGoalMotion_;

  // \TODO Make this variable unnecessary, or at least have it
  // persist across solve runs
  base::Cost bestCost = opt_->infiniteCost();

  bestCost_ = opt_->infiniteCost();

  Motion* approximation = nullptr;
  double approximatedist = std::numeric_limits<double>::infinity();
  bool sufficientlyShort = false;

  Motion* rmotion = new Motion(si_);
  base::State* rstate = rmotion->state;
  base::State* xstate = si_->allocState();

  // e+e/d.  K-nearest RRT*
  double k_rrg = boost::math::constants::e<double>() +
                 (boost::math::constants::e<double>() /
                  (double)si_->getStateSpace()->getDimension());

  std::vector<Motion*> nbh;
  std::queue<Motion*> recover;
  bool recoverFlag = 0;

  std::vector<base::Cost> costs;
  std::vector<base::Cost> incCosts;
  std::vector<std::size_t> sortedCostIndices;

  std::vector<int> valid;
  unsigned int rewireTest = 0;
  unsigned int statesGenerated = 0;

  bool approximate = true;
  bool addedSolution = false;

  if (solution)
    OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f",
                getName().c_str(), solution->cost.value());
  OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
              (unsigned int)std::ceil(k_rrg * log((double)(nn_->size() + 1))));

  // our functor for sorting nearest neighbors
  CostIndexCompare compareFn(costs, *opt_);

  OMPL_INFORM("size of the tree %d", nn_->size());

  // XXX this should be a cheap operation
  std::function<bool(Motion*)> majorTree;
  majorTree = [&](Motion* m) -> bool {
    if (m->parent == nullptr && m->nodeType == NORMAL)
      return true;
    else if (m->nodeType == ORPHANED)
      return false;
    else
      return majorTree(m->parent);
  };

  auto isOnPath = [&](Motion* m) -> bool {
    for (auto& orphanState : orphanedBiasNodes_) {
      if (si_->equalStates(m->state, orphanState)) {
        return true;
      }
    }
    return false;
  };

  while (ptc() == false) {
    recoverFlag = false;
    iterations_++;

    // sample random state (with goal biasing)
    // Goal samples are only sampled until maxSampleCount() goals are in the
    // tree, to prohibit duplicate goal states.
    if (localPlanning_) {
      if (recover.size() == 0) {
        if (rng_.uniform01() < orphanedBias_ && orphanedBiasNodes_.size() > 0) {
          size_t whereSample =
              rng_.uniformInt(0, orphanedBiasNodes_.size() - 1);
          sampler_->sampleUniformNear(rstate, orphanedBiasNodes_[whereSample],
                                      sampleRadius_);
        } else {
          if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() &&
              rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
          else
            sampler_->sampleUniform(rstate);
        }
      } else {
        Motion* t = recover.front();
        si_->copyState(rstate, t->state);
        recoverFlag = true;
      }
    } else {
      if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() &&
          rng_.uniform01() < goalBias_ && goal_s->canSample())
        goal_s->sampleGoal(rstate);
      else
        sampler_->sampleUniform(rstate);
    }

    // find closest state in the tree
    Motion* nmotion = nn_->nearest(rmotion);

    if (nmotion->nodeType == ORPHANED || nmotion->nodeType == INVALID ||
        !majorTree(nmotion)) {
#ifdef DEBUG
      OMPL_ERROR("tried to connect to an ORPHANED node %d\n",
                 nmotion->nodeType);
      OMPL_ERROR("tried to connect to a REMOVED node %d\n", nmotion->nodeType);
#endif
      std::vector<Motion*> my_motions;
      nn_->nearestK(rmotion, k_rrg * 20, my_motions);
      size_t j = 0;
      for (; j < my_motions.size(); ++j) {
        if (my_motions[j]->nodeType != NodeType::ORPHANED &&
            my_motions[j]->nodeType != NodeType::INVALID &&
            majorTree(my_motions[j]))
          break;
      }
      if (j >= my_motions.size())
        continue;  // skip if every neighbor is invalid
      nmotion = my_motions[j];
    }

    if (intermediateSolutionCallback &&
        si_->equalStates(nmotion->state, rstate))
      continue;

    base::State* dstate = rstate;

    // find state to add to the tree
    double d = si_->distance(nmotion->state, rstate);
    if (d > maxDistance_ && !recoverFlag) {
      // OMPL_INFORM("d = %f, max = %f, max/d = %f", d, maxDistance_,
      // maxDistance_ / d);
      si_->getStateSpace()->interpolate(nmotion->state, rstate,
                                        maxDistance_ / d, xstate);
      // OMPL_INFORM("upd d = %f, max = %f, max/d = %f", d, maxDistance_,
      // maxDistance_ / d);
      dstate = xstate;
    }

    // Check if the motion between the nearest state and the state to add is
    // valid
    if (si_->checkMotion(nmotion->state, dstate)) {
      // create a motion
      Motion* motion = nullptr;
      if (!recoverFlag) {
        motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
      }
      if (recoverFlag) {
        motion = recover.front();
        recover.pop();
        if (motion->parent->nodeType != ORPHANED)
          removeFromParent(motion);
        else if (motion->nodeType != ORPHANED) {
          auto& children = motion->parent->children;
          auto toRemove = std::find(children.begin(), children.end(), motion);
          children.erase(toRemove);
        }

        for (auto my_iter = orphanedBiasNodes_.begin();
             my_iter != orphanedBiasNodes_.end(); ++my_iter) {
          ompl::base::State* st = *my_iter;
          if (si_->equalStates(st, motion->state)) {
            orphanedBiasNodes_.erase(my_iter);
            break;
          }
        }
      }
      motion->parent = nmotion;
      motion->incCost = opt_->motionCost(nmotion->state, motion->state);
      motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

      // Find nearby neighbors of the new motion - k-nearest RRT*
      unsigned int k = std::ceil(k_rrg * log((double)(nn_->size() + 1)));
      if (bakNN_ != nullptr && bakNN_->size() > nn_->size()) {
        k = std::ceil(k_rrg * log((double)(2 * bakNN_->size() + 1)));
      }
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
        for (auto i = sortedCostIndices.begin();
             i != sortedCostIndices.begin() + nbh.size(); ++i) {
          // skip orphaned nodes at this stage
          if (nbh[*i]->nodeType == NodeType::ORPHANED ||
              nbh[*i]->nodeType == NodeType::INVALID)
            continue;

          if (!majorTree(nbh[*i])) continue;

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
          // skip orphaned nodes at this stage
          if (nbh[i]->nodeType == NodeType::ORPHANED ||
              nbh[i]->nodeType == NodeType::INVALID)
            continue;
          if (!majorTree(nbh[i])) continue;

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

      // add motion to the tree
      if (!recoverFlag) {
        nn_->add(motion);
      } else {
        OMPL_INFORM(">>> reunited the branch");
        motion->nodeType = NORMAL;
        ptc.terminate();
      }
      motion->parent->children.push_back(motion);

      bool checkForSolution = false;
      for (std::size_t i = 0; i < nbh.size(); ++i) {
        if (nbh[i] != motion->parent &&
            (nbh[i]->nodeType == NORMAL && majorTree(nbh[i]))) {
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

        if (nbh[i]->nodeType == ORPHANED ||
            (isOnPath(nbh[i]) && !majorTree(nbh[i]))) {
          // OMPL_INFORM("> about to connect an orphan branch");
          if (si_->checkMotion(motion->state, nbh[i]->state)) {
            recover.push(nbh[i]);
            checkForSolution = true;
            break;
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
            std::vector<const base::State*> spath;
            Motion* intermediate_solution =
                solution
                    ->parent;  // Do not include goal state to simplify code.

            do {
              spath.push_back(intermediate_solution->state);
              intermediate_solution = intermediate_solution->parent;
            } while (intermediate_solution->parent !=
                     0);  // Do not include the start state.

            intermediateSolutionCallback(this, spath, bestCost_);
          }

          approximate = (solution == nullptr);
          addedSolution = false;
          if (approximate)
            solution = approximation;
          else
            lastGoalMotion_ = solution;

          if (solution != nullptr) {
            ptc.terminate();
            // construct the solution path
            std::vector<Motion*> mpath;
            while (solution != nullptr) {
              std::vector<Motion*>::iterator it;
              it = find(mpath.begin(), mpath.end(), solution);
              if (mpath.end() != it) {
                OMPL_WARN("cycle detected, this solution may be invalid");
                break;
              }
              mpath.push_back(solution);
              if (solution == solution->parent) {
                OMPL_WARN("the solution is in the orphaned branch");
              }
              solution = solution->parent;
            }

            // set the solution path
            PathGeometric* geoPath = new PathGeometric(si_);
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
        }
      }

      // Checking for approximate solution (closest state found to the goal)
      if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist) {
        approximation = motion;
        approximatedist = distanceFromGoal;
      }

      if (nn_->size() >= maxNodes_) {
#ifdef DEBUG
        OMPL_INFORM("%d > %d", statesGenerated, maxNodes_);
#endif
        // FIXME #39 this is a bottleneck
        std::vector<Motion*> motions;
        nn_->list(motions);
        std::vector<std::size_t> childlessNodes;
        for (std::size_t i = 0; i < motions.size(); ++i) {
          if (motions[i]->children.size() == 0) childlessNodes.push_back(i);
        }
#ifdef DEBUG
        OMPL_INFORM("childless num %d", childlessNodes.size());
#endif

        if (childlessNodes.size() > 0) {
          std::size_t rmNode = rng_.uniformInt(0, childlessNodes.size() - 1);
          // removing information about the child in the parent node
          // did not have a chance to check this line
          while (approximation == motions[childlessNodes[rmNode]])
            rmNode = rng_.uniformInt(0, childlessNodes.size() - 1);

          removeFromParent(motions[childlessNodes[rmNode]]);
          // removing
          bool rmResult = nn_->remove(motions[childlessNodes[rmNode]]);
          if (rmResult == false)
            OMPL_WARN("cannot remove the node");
          else
            removedNodes++;
        } else {
          OMPL_WARN("zero childless nodes");
        }
      }
    }

    // terminate if a sufficient solution is found
    if (solution && sufficientlyShort) break;
  }

  while (recover.size() != 0) recover.pop();

  si_->freeState(xstate);
  if (rmotion->state) si_->freeState(rmotion->state);
  delete rmotion;

  OMPL_INFORM(
      "%s: Created %u new states. Checked %u rewire options. %u goal states in"
      "tree.",
      getName().c_str(), statesGenerated, rewireTest, goalMotions_.size());

  OMPL_INFORM("Number of removed nodes is %d", removedNodes);
  return base::PlannerStatus(addedSolution, approximate);
}

void ompl::geometric::DRRTstarFN::evaluateSolutionPath() {
  std::function<bool(Motion*)> majorTree;
  majorTree = [&](Motion* m) -> bool {
    if (m->parent == nullptr && m->nodeType == NORMAL)
      return true;
    else if (m->nodeType == ORPHANED)
      return false;
    else
      return majorTree(m->parent);
  };

  std::vector<Motion*> motions;
  nn_->list(motions);
  if (goalMotions_.size() > 1) {
    OMPL_WARN(
        "I did not think that it would be possible, goal motions count: %d",
        goalMotions_.size());
  }

  Motion* m = nullptr;
  if (goalMotions_.size() > 0)
    m = goalMotions_.back();
  else {
    OMPL_WARN(
        "Goal state is not in the tree providing solution with an "
        "approximation");

    assert(pdef_ && "Problem definition is null");
    Motion* goal_motion = new Motion(si_);
    si_->copyState(goal_motion->state,
                   pdef_->getGoal()->as<ompl::base::GoalState>()->getState());
    m = nn_->nearest(goal_motion);
  }

  std::vector<Motion*> solutions;
  nn_->nearestK(m, 20, solutions);
  size_t j = 0;
  for (; j < solutions.size(); ++j)
    if (majorTree(solutions[j])) break;

  if (j >= solutions.size()) {
    OMPL_WARN("No solution");
    return;
  }

  auto solution = solutions[j];

  pdef_->clearSolutionPaths();

  if (solution != nullptr) {
    // construct the solution path
    std::vector<Motion*> mpath;
    while (solution != nullptr) {
      std::vector<Motion*>::iterator it;
      it = find(mpath.begin(), mpath.end(), solution);
      if (mpath.end() != it) {
        OMPL_WARN("cycle detected, this solution may be invalid");
        break;
      }
      mpath.push_back(solution);
      if (solution == solution->parent) {
        OMPL_WARN("the solution is in the orphaned branch");
      }
      solution = solution->parent;
    }

    // set the solution path
    PathGeometric* geoPath = new PathGeometric(si_);
    for (int i = mpath.size() - 1; i >= 0; --i)
      geoPath->append(mpath[i]->state);

    base::PathPtr path(geoPath);
    // Add the solution path.
    base::PlannerSolution psol(path);
    psol.setPlannerName(getName());
    auto bestCost = solutions[j]->cost;
    psol.setOptimized(opt_, bestCost, 1);
    pdef_->addSolutionPath(psol);
  }
}

void ompl::geometric::DRRTstarFN::removeFromParent(Motion* m) {
  if (m->parent == nullptr) return;

  auto it = std::find(std::begin(m->parent->children),
                      std::end(m->parent->children), m);

  if (it != std::end(m->parent->children)) m->parent->children.erase(it);
}

void ompl::geometric::DRRTstarFN::updateChildCosts(Motion* m) {
  for (std::size_t i = 0; i < m->children.size(); ++i) {
    m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
    updateChildCosts(m->children[i]);
  }
}

void ompl::geometric::DRRTstarFN::freeMemory() {
  if (nn_) {
    std::vector<Motion*> motions;
    nn_->list(motions);
    for (std::size_t i = 0; i < motions.size(); ++i) {
      if (motions[i]->state) si_->freeState(motions[i]->state);
      delete motions[i];
    }
  }
}

void ompl::geometric::DRRTstarFN::getPlannerData(
    base::PlannerData& data) const {
  std::function<bool(Motion*)> isMajorTree;
  isMajorTree = [&](Motion* m) -> bool {

    if (m == nullptr) return true;

    if (m->parent == nullptr && m->nodeType == NORMAL)
      return true;
    else if (m->nodeType == ORPHANED)
      return false;
    else
      return isMajorTree(m->parent);
  };

  Planner::getPlannerData(data);

  std::vector<Motion*> motions;

  nn_->list(motions);

  if (lastGoalMotion_)
    data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

  for (std::size_t i = 0; i < motions.size(); ++i) {
    if (motions[i]->parent == nullptr && isMajorTree(motions[i])) {
      base::PlannerDataVertex rootVertex(motions[i]->state);
      // major tree by default
      rootVertex.setTag(true);
      data.addStartVertex(rootVertex);
    } else {
      base::PlannerDataVertex myVertex(motions[i]->state);

      myVertex.setTag(isMajorTree(motions[i]));

      if (motions[i]->parent != nullptr) {
        base::PlannerDataVertex myVertexParent(motions[i]->parent->state);

        myVertexParent.setTag(isMajorTree(motions[i]->parent));

        data.addEdge(myVertexParent, myVertex);
      }
    }
  }
}

void ompl::geometric::DRRTstarFN::setPlannerData(
    const ompl::base::PlannerData& data) {
  DynamicPlanner::setPlannerData(data);
  std::vector<Motion*> motions;

  for (unsigned int i = 0; i != data.numVertices(); ++i) {
    ompl::base::PlannerDataVertex v = data.getVertex(i);
    Motion* m = new Motion(si_);
    si_->copyState(m->state, v.getState());
    m->parent = nullptr;
    motions.push_back(m);

    if (data.isStartVertex(i)) {
      pdef_->setStartAndGoalStates(v.getState(), v.getState());
      OMPL_INFORM(">>> there is a start state in the tree");
    }

    if (data.isGoalVertex(i)) {
      OMPL_INFORM(">>> there is a goal state in the tree");
      pdef_->setGoalState(v.getState());
    }
  }

  for (unsigned int i = 0; i != data.numEdges(); ++i) {
    std::vector<unsigned int> edgeList;
    data.getEdges(i, edgeList);

    for (const auto& num : edgeList) {
      motions[i]->children.push_back(motions[num]);
      motions[num]->parent = motions[i];
    }
  }

  std::for_each(motions.begin(), motions.end(),
                [&](Motion*& m) { nn_->add(m); });

  base::Goal* goal = pdef_->getGoal().get();
  // Add the new motion to the goalMotion_ list, if it satisfies the goal
  double distanceFromGoal;

  bool checkForSolution = false;
  // TODO adapt this for the place here.
/*
  std::for_each(motions.begin(), motions.end(), [&](Motion* motion) {

    // \TODO Make this variable unnecessary, or at least have it
    // persist across solve runs
    base::Cost bestCost = opt_->infiniteCost();

    bestCost_ = opt_->infiniteCost();

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
          std::vector<const base::State*> spath;
          Motion* intermediate_solution =
              solution->parent;  // Do not include goal state to simplify
          code.

              do {
            spath.push_back(intermediate_solution->state);
            intermediate_solution = intermediate_solution->parent;
          }
          while (intermediate_solution->parent != 0)
            ;  // Do not include the start state.

          intermediateSolutionCallback(this, spath, bestCost_);
        }

        approximate = (solution == nullptr);
        addedSolution = false;
        if (approximate)
          solution = approximation;
        else
          lastGoalMotion_ = solution;

        if (solution != nullptr) {
          ptc.terminate();
          // construct the solution path
          std::vector<Motion*> mpath;
          while (solution != nullptr) {
            std::vector<Motion*>::iterator it;
            it = find(mpath.begin(), mpath.end(), solution);
            if (mpath.end() != it) {
              OMPL_WARN("cycle detected, this solution may be invalid");
              break;
            }
            mpath.push_back(solution);
            if (solution == solution->parent) {
              OMPL_WARN("the solution is in the orphaned branch");
            }
            solution = solution->parent;
          }

          // set the solution path
          PathGeometric* geoPath = new PathGeometric(si_);
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
      }
    }

  });
  */
}
//==============================================================================
ompl::base::Cost ompl::geometric::DRRTstarFN::costToGo(
    const Motion* motion, const bool shortest) const {
  base::Cost costToCome;
  if (shortest)
    costToCome = opt_->motionCost(startMotion_->state, motion->state);  // h_s
  else
    costToCome = motion->cost;  // d_s

  const base::Cost costToGo =
      base::goalRegionCostToGo(motion->state, pdef_->getGoal().get());  // h_g
  return opt_->combineCosts(costToCome, costToGo);  // h_s + h_g
}
//==============================================================================
void ompl::geometric::DRRTstarFN::restoreTree(const std::string& filename) {
  restoreTree(filename.c_str());
}
//==============================================================================
void ompl::geometric::DRRTstarFN::restoreTree(const char* filename) {
  checkValidity();

  ompl::base::PlannerData pdat(si_);
  ompl::base::PlannerDataStorage pdstorage;
  pdstorage.load(filename, pdat);

  // find a root node of the tree and add nodes to the tree accordingly
  for (unsigned int i = 0; i != pdat.numVertices(); ++i) {
    if (pdat.isStartVertex(i)) {
      traverseTree(i, pdat);
      break;
    }
  }
}

bool ompl::geometric::DRRTstarFN::traverseTree(
    const unsigned int n, const ompl::base::PlannerData& pdat) {
  const base::ReportIntermediateSolutionFn intermediateSolutionCallback =
      pdef_->getIntermediateSolutionCallback();
  Motion* solution = lastGoalMotion_;
  base::Cost bestCost = opt_->infiniteCost();

  bestCost_ = opt_->infiniteCost();

  Motion* approximation = nullptr;
  double approximatedist = std::numeric_limits<double>::infinity();
  bool sufficientlyShort = false;

  base::Goal* goal = pdef_->getGoal().get();
  // base::GoalSampleableRegion* goal_s =
  // dynamic_cast<base::GoalSampleableRegion*>(goal);
  // std::vector<Motion*> orphans;

  // bool symCost = opt_->isSymmetric();

  std::vector<unsigned int> edgeList;
  std::deque<unsigned int> queue;
  std::deque<Motion*> nodeList;

  queue.push_back(n);

  Motion* motion = new Motion(si_);
  motion->cost = opt_->identityCost();
  si_->copyState(motion->state, pdat.getVertex(queue.front()).getState());
  nn_->add(motion);
  startMotion_ = motion;

  if (nn_->size() == 0) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  nodeList.push_back(motion);

  int nodesAdded = 0;

  while (!queue.empty()) {
    pdat.getEdges(queue.front(), edgeList);
    for (size_t i = 0; i < edgeList.size(); ++i) {
      motion = new Motion(si_);
      si_->copyState(motion->state, pdat.getVertex(edgeList[i]).getState());
      motion->parent = nodeList.front();
      motion->incCost =
          opt_->motionCost(nodeList.front()->state, motion->state);
      motion->cost =
          opt_->combineCosts(nodeList.front()->cost, motion->incCost);
      nn_->add(motion);
      nodesAdded++;
      motion->parent->children.push_back(motion);
      nodeList.push_back(motion);
      queue.push_back(edgeList[i]);

      // Add the new motion to the goalMotion_ list, if it satisfies the goal
      double distanceFromGoal;
      bool checkForSolution = false;

      if (goal->isSatisfied(motion->state, &distanceFromGoal)) {
        goalMotions_.push_back(motion);
        checkForSolution = true;
      }

      // Checking for approximate solution (closest state found to the goal)
      if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist) {
        approximation = motion;
        approximatedist = distanceFromGoal;
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
            std::vector<const base::State*> spath;
            Motion* intermediate_solution =
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
    }

    nodeList.pop_front();
    queue.pop_front();
  }
  bool approximate = (solution == nullptr);
  if (approximate)
    solution = approximation;
  else
    lastGoalMotion_ = solution;

  if (solution != nullptr) {
    // construct the solution path
    std::vector<Motion*> mpath;
    while (solution != nullptr) {
      mpath.push_back(solution);
      solution = solution->parent;
    }

    // set the solution path
    PathGeometric* geoPath = new PathGeometric(si_);
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
  }
  return true;
}

// decouple it later

void ompl::geometric::DRRTstarFN::selectBranch(ompl::base::State* s) {
  std::vector<Motion*> tree;
  nn_->list(tree);

  subTreeNN_.reset(new NearestNeighborsLinear<Motion*>());
  //
  subTreeNN_.reset(
      tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));

  subTreeNN_->setDistanceFunction(std::bind(&DRRTstarFN::distanceFunction, this,
                                            std::placeholders::_1,
                                            std::placeholders::_2));

  auto start = std::find_if(tree.begin(), tree.end(), [&](Motion* m) -> bool {
    return si_->equalStates(m->state, s);
  });

  if (start != tree.end()) {
    std::function<void(Motion*)> setSubTree;
    setSubTree = [&](Motion* m) {
      subTreeNN_->add(m);
      // m->nodeType = NEW_ORPHANED;
      for (auto child : m->children) {
        //        std::future<void> f =
        //            std::async(std::launch::async, markNewOrphaned, child);
        setSubTree(child);
      }
    };

    Motion* m = *start;
    removeFromParent(m);
    m->parent = nullptr;
    setSubTree(m);
    // std::async(std::launch::async, markNewOrphaned, m);
  }
}
//==============================================================================
std::size_t ompl::geometric::DRRTstarFN::removeInvalidNodes() {
  const static int LIMIT_PATH = 2500;
  std::size_t removed = 0;
  int error_removed = 0;

  if (goalMotions_.size() == 0) {
    OMPL_WARN("No goal state is not in the tree");
  }

  std::vector<Motion*> nbh;

  Motion* node = nullptr;
  if (goalMotions_.size() > 0)
    node = goalMotions_.back();
  else {
    // if (!pdef_) {
    Motion* temp = new Motion(si_);
    si_->copyState(temp->state,
                   pdef_->getGoal()->as<ompl::base::GoalState>()->getState());
    node = nn_->nearest(temp);
    //}
  }

  // think about error handling
  std::function<void(Motion*)> removeBranch;
  removeBranch = [&](Motion* m) {
    for (auto& child : m->children) {
      removeBranch(child);
      // assert(removeBranch(child));
    }
    if (nn_->remove(m))
      removed++;
    else {
      error_removed++;
    }
  };

  // e+e/d.  K-nearest RRT*
  // double k_rrg = boost::math::constants::e<double>() +
  // (boost::math::constants::e<double>() /
  // (double)si_->getStateSpace()->getDimension());
  // Find nearby neighbors of the new motion - k-nearest RRT*
  // unsigned int k = std::ceil(k_rrg * log((double)(bakNN_->size() + 1)));
  // unsigned int k = std::ceil(nn_->size() * 0.2);

  // this is a variable to control the removal procedure

  std::vector<Motion*> sPath;
  std::vector<Motion*> faultyMotions;

  auto isOnPath = [&](Motion* m) -> bool {
    for (auto& motion : sPath) {
      if (si_->equalStates(motion->state, m->state)) return true;
    }
    return false;
  };

  for (size_t watchdog = 0; node->parent != nullptr && watchdog < LIMIT_PATH;
       ++watchdog, node = node->parent) {
    sPath.push_back(node);

    if (!si_->isValid(node->parent->state)) node->parent->nodeType = INVALID;

    if (!si_->checkMotion(node->parent->state, node->state)) {
      Motion* m = new Motion(si_);
      m->state = si_->allocState();
      si_->getStateSpace()->interpolate(node->parent->state, node->state, 0.5,
                                        m->state);

      faultyMotions.push_back(m);

      removeFromParent(node);
    }

    if (!si_->isValid(node->state)) node->nodeType = INVALID;
  }

  for (int i = static_cast<int>(sPath.size()) - 1; i >= 0; --i) {
    if (sPath[i]->nodeType == INVALID) {
      removeFromParent(sPath[i]);
      for (auto& child : sPath[i]->children) {
        if (child != sPath[i - 1]) {
          removeFromParent(child);
          removeBranch(child);
        }
      }
      nn_->remove(sPath[i]);
      sPath[i - 1]->nodeType = ORPHANED;
    }
  }

  orphanedBiasNodes_.clear();
  std::vector<Motion*> motions;
  nn_->list(motions);
  for (auto& m : faultyMotions) {
    nbh.clear();

    std::cout << m->nodeType % 1;  // placed here to ignore m is unused

    // removeAssistanceCallback();
    /*
 for (auto& motion : motions) {
    const ompl::base::SE2StateSpace::StateType* s1
        = motion->state->as<ompl::base::SE2StateSpace::StateType>();
    const ompl::base::SE2StateSpace::StateType* s2
        = m->state->as<ompl::base::SE2StateSpace::StateType>();

    double dx = s1->getX() - s2->getX();
    double dy = s1->getY() - s2->getY();

    double radius = 450;
    if (dx * dx + dy * dy < radius * radius)
        nbh.push_back(motion);
}
*/

    for (auto& neighbor : nbh) {
      if (neighbor->parent != nullptr) {
        if (!si_->isValid(neighbor->state)) {
          removeFromParent(neighbor);
          if (!isOnPath(neighbor)) removeBranch(neighbor);
          nn_->remove(neighbor);
          continue;
        }

        if ((neighbor->parent != neighbor) &&
            !si_->checkMotion(neighbor->parent->state, neighbor->state)) {
          removeFromParent(neighbor);
          if (!isOnPath(neighbor))
            removeBranch(neighbor);
          else {
            neighbor->parent = neighbor;
            neighbor->nodeType = ORPHANED;

            for (auto& pathNode : sPath) {
              orphanedBiasNodes_.push_back(pathNode->state);
              if (si_->equalStates(neighbor->state, pathNode->state)) break;
            }
          }
        }
      }
    }
  }

  OMPL_WARN("failed to remove nodes from the nn_ %d points", error_removed);
  return removed;
}
//==============================================================================
void ompl::geometric::DRRTstarFN::nodeCleanUp(ompl::base::State* s) {
  std::vector<Motion*> motions;
  nn_->list(motions);

  std::function<void(Motion*)> rmBranch;
  rmBranch = [&](Motion* m) {
    nn_->remove(m);
    for (auto child : m->children) {
      rmBranch(child);
    }
  };

  std::function<bool(Motion*)> majorTree;
  majorTree = [&](Motion* m) -> bool {
    if (m->parent == nullptr && m->nodeType == NORMAL)
      return true;
    else if (m->nodeType == ORPHANED)
      return false;
    else
      return majorTree(m->parent);
  };

  for (auto m : motions) {
    if (!majorTree(m)) {
      if (!si_->equalStates(m->state, s)) {
        rmBranch(m);
      }
    }
  }
}

void ompl::geometric::DRRTstarFN::setPreviousPath(
    std::vector<ompl::base::State*> stateList, int stateIndex) {
  orphanedBiasNodes_.clear();
  for (auto it = stateList.begin() + stateIndex; it < stateList.end(); ++it) {
    ompl::base::State* s = *it;
    orphanedBiasNodes_.push_back(si_->cloneState(s));
  }
}

void ompl::geometric::DRRTstarFN::populateDetachedPath() {
  if (goalMotions_.size() == 0) {
    OMPL_WARN("No goal state is not in the tree");
  }

  std::vector<Motion*> nbh;

  Motion* node = nullptr;
  if (goalMotions_.size() > 0)
    node = goalMotions_.back();
  else {
    // if (!pdef_) {
    Motion* temp = new Motion(si_);
    si_->copyState(temp->state,
                   pdef_->getGoal()->as<ompl::base::GoalState>()->getState());
    node = nn_->nearest(temp);
    //}
  }

  const int steps = 25;
  const double step = 1.0 / static_cast<double>(steps);

  while (node->nodeType != ORPHANED) {
    Motion* array[steps];
    for (int i = 0; i < steps; ++i) {
      Motion* m = new Motion(si_);
      m->state = si_->allocState();
      si_->getStateSpace()->interpolate(node->parent->state, node->state,
                                        (i + 1) * step, m->state);

      array[i] = m;
      nn_->add(m);
      orphanedBiasNodes_.push_back(m->state);
    }
    array[0]->parent = node->parent;
    node->parent->children.push_back(array[0]);
    removeFromParent(node);

    for (int i = 1; i < steps; ++i) {
      array[i]->parent = array[i - 1];
      array[i - 1]->children.push_back(array[i]);
    }

    node->parent = array[steps - 1];
    node->parent->children.push_back(node);

    node = array[0]->parent;
  }
}

void ompl::geometric::DRRTstarFN::swapNN() {
  bakNN_ = nn_;
  nn_ = subTreeNN_;
}

bool ompl::geometric::DRRTstarFN::reconnect() {
  // XXX this should be a cheap operation
  std::function<bool(Motion*)> majorTree;
  majorTree = [&](Motion* m) -> bool {
    if (m->parent == nullptr && m->nodeType == NORMAL)
      return true;
    else if (m->nodeType == ORPHANED)
      return false;
    else
      return majorTree(m->parent);
  };

  Motion* node = nullptr;
  if (goalMotions_.size() > 0)
    node = goalMotions_.back();
  else
    return false;

  size_t k = 10;
  while (node->nodeType != ORPHANED) {
    std::vector<Motion*> nbh;
    nn_->nearestK(node, k, nbh);
    for (auto& neighbor : nbh) {
      if (majorTree(neighbor)) {
        if (si_->checkMotion(neighbor->state, node->state)) {
          base::Cost nbhIncCost;
          nbhIncCost = opt_->motionCost(neighbor->state, node->state);
          base::Cost nbhNewCost = opt_->combineCosts(node->cost, nbhIncCost);

          node->parent = neighbor;
          node->incCost = nbhIncCost;
          node->cost = nbhNewCost;
          node->parent->children.push_back(node);
          node->nodeType = NORMAL;

          updateChildCosts(node);
          OMPL_INFORM("reconnected");
          return true;
        }
      }
    }
    node = node->parent;
  }
  return false;
}

void ompl::geometric::DRRTstarFN::prepareDynamic(std::size_t from) {
  std::vector<ompl::base::State*> pathArray_;
  try {
    ompl::base::SpaceInformationPtr si = getSpaceInformation();
    PathGeometric& p = static_cast<PathGeometric&>(*(pdef_->getSolutionPath()));
    setRange(35.0);
    ompl::base::State* s = si->cloneState(p.getState(from));
    p.interpolate();
    for (size_t i = from; i < p.getStates().size(); ++i) {
      ompl::base::State* st = p.getState(i);
      pathArray_.push_back(si->cloneState(st));
    }

    si->freeState(s);
    selectBranch(s);

    setSampleRadius(1000);
    setOrphanedBias(0.65);
    setLocalPlanning(true);
    swapNN();
  } catch (ompl::Exception e) {
    OMPL_ERROR("No solution, man\n");
  }
}
