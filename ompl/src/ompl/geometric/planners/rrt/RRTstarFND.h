#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRTSTARFND_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRTSTARFND_

#include "ompl/base/DynamicPlanner.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/planners/rrt/RRTstarFN.h"

#include <limits>
#include <list>
#include <tuple>
#include <utility>
#include <vector>

namespace ompl {
namespace geometric {

/** \brief Optimal Rapidly-exploring Random Trees */
class RRTstarFND : public base::DynamicPlanner {
  friend class boost::serialization::access;

 public:
  ///
  /// \brief constructor
  /// \param si
  ///
  RRTstarFND(const base::SpaceInformationPtr& si);

  /// \brief destructor
  virtual ~RRTstarFND();

  /// \brief
  virtual void prepare() { ; }
  /// \brief
  virtual void react();
  /// \brief
  virtual void prePause() { ; }
  /// \brief
  virtual void postPause() { ; }
  /// \brief
  virtual void preReact();
  /// \brief
  virtual void postReact() { ; }
  /// \brief
  virtual void preMove() { ; }
  /// \brief
  virtual void postMove() { ; }

  /**
   * @brief getPlannerData
   * @param data
   */
  virtual void getPlannerData(base::PlannerData& data) const;

  /**
   * @brief setPlannerData
   * @param data
   */
  virtual void setPlannerData(const base::PlannerData& data);

  /**
   * @brief clear
   */
  virtual void clear();

  /**
   * @brief solve
   * @param ptc
   * @return
   */
  virtual base::PlannerStatus solve(
      const base::PlannerTerminationCondition& ptc);

  /** \brief Set the goal bias

      In the process of randomly selecting states in
      the state space to attempt to go towards, the
      algorithm may in fact choose the actual goal state, if
      it knows it, with some probability. This probability
      is a real number between 0.0 and 1.0; its value should
      usually be around 0.05 and should not be too large. It
      is probably a good idea to use the default value. */
  void setGoalBias(double goalBias) { goalBias_ = goalBias; }

  /** \brief Get the goal bias the planner is using */
  double getGoalBias() const { return goalBias_; }

  /** \brief fraction of the time in the dynamic part of
   * the motion planning will be biased with the following variable */
  void setOrphanedBias(double orphanedBias) { orphanedBias_ = orphanedBias; }

  /** \brief  get the biasing factor for orphaned nodes*/
  double getOrphanedBias() { return orphanedBias_; }

  /** \brief Set the range the planner is supposed to use.

    This parameter greatly influences the runtime of the
    algorithm. It represents the maximum length of a
    motion to be added in the tree of motions. */
  void setRange(double distance) { maxDistance_ = distance; }

  /** \brief Get the range the planner is using */
  double getRange() const { return maxDistance_; }

  ///
  /// \brief setPreviousPath
  /// \param stateList
  /// \param stateIndex
  ///
  void setPreviousPath(std::vector<ompl::base::State*> stateList,
                       int stateIndex);
  ///
  /// \brief nodeCleanUp
  /// \param s
  ///
  void nodeCleanUp(ompl::base::State* s);

  ///
  /// \brief setup
  ///
  virtual void setup();

  ///////////////////////////////////////
  // Planner progress property functions
  std::string getIterationCount() const {
    return boost::lexical_cast<std::string>(iterations_);
  }
  std::string getBestCost() const {
    return boost::lexical_cast<std::string>(bestCost_);
  }

  ///
  /// \brief setMaxNodes
  /// \param maxNodes
  ///
  void setMaxNodes(const std::size_t& maxNodes) { maxNodes_ = maxNodes; }

  ///
  /// \brief getMaxNodes
  /// \return
  ///
  std::size_t getMaxNodes() const { return maxNodes_; }

  /// \brief
  void setLocalPlanning(bool set) { localPlanning_ = set; }

  /// \brief Set sampling radius around interim state
  bool isLocalPlanning() { return localPlanning_; }

  /// \brief Set the interim state
  void setInterimState(base::State* state) { interimState_ = state; }

  /// \brief Set sampling radius around the interim state
  void setSampleRadius(double r) { sampleRadius_ = r; }

  /// \brief Remove the states from the tree
  std::size_t removeInvalidNodes();

  /// \brief
  void prepareDynamic(std::size_t from);

  /// \brief
  void evaluateSolutionPath();

  /** \brief interpolates the detached solution path and adds nodes in between
   * to the tree */
  void populateDetachedPath();

  /** \brief remove orphaned nodes from the tree */
  void removeOrphaned();

  /** \brief reconnect orphaned nodes to the tree */
  bool reconnect();

  /** \brief selects the branch */
  void selectBranch(ompl::base::State* s);

  /// \brief
  void swapNN();

  /** \brief Save the state of the tree */
  void restoreTree(const std::string& filename);

  /** \brief Save the state of the tree */
  void restoreTree(const char* filename);

  // /** \brief Load the state of the tree */
  // void loadTree(const char *filename);

  enum NodeType : char { NORMAL = 0, ORPHANED = 1, INVALID = 2 };

 protected:
  /** \brief Representation of a motion */
  class Motion {
   public:
    /** \brief Constructor that allocates memory for the state. This
    constructor
     * automatically allocates memory for \e state, \e cost, and \e incCost
     */
    Motion(const base::SpaceInformationPtr& si)
        : state(si->allocState()),
          parent(nullptr),
          nodeType(NodeType::NORMAL) {}

    ~Motion() { ; }

    /** \brief The state contained by the motion */
    base::State* state;

    /** \brief The parent motion in the exploration tree */
    Motion* parent;

    /** \brief The cost up to this motion */
    base::Cost cost;

    /** \brief The incremental cost of this motion's parent to this motion
    (this
     * is stored to save distance computations in the updateChildCosts()
     method)
     */
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
        : costs_(costs), opt_(opt) {}
    bool operator()(unsigned i, unsigned j) {
      return opt_.isCostBetterThan(costs_[i], costs_[j]);
    }
    const std::vector<base::Cost>& costs_;
    const base::OptimizationObjective& opt_;
  };

  /** \brief Compute distance between motions (actually distance between
   * contained states) */
  double distanceFunction(const Motion* a, const Motion* b) const {
    return si_->distance(a->state, b->state);
  }

  /** \brief Removes the given motion from the parent's child list */
  void removeFromParent(Motion* m);

  /** \brief Updates the cost of the children of this node if the cost up to
   * this node has changed */
  void updateChildCosts(Motion* m);

  /// \brief
  void verifyTree();

  //  bool switchToDynamic();

  /** \brief this function is used for reconstruction of the tree from the
  file
   */
  bool traverseTree(const unsigned int n, const ompl::base::PlannerData& pdat);

  /** \brief Computes the Cost To Go heuristically as the cost to come from
    start to motion plus
    the cost to go from motion to goal. If \e shortest is true, the
    estimated
    cost to come
    start-motion is given. Otherwise, this cost to come is the current
    motion
    cost. */
  base::Cost costToGo(const Motion* motion, const bool shortest = true) const;

  /** \brief State sampler */
  base::StateSamplerPtr sampler_;

  /** \brief A nearest-neighbors datastructure containing the tree of
  motions */
  std::shared_ptr<NearestNeighbors<Motion*> > nn_;

  /** \brief A nearest-neighbors datastructure containing the subtree of
  motions
   */
  std::shared_ptr<NearestNeighbors<Motion*> > subTreeNN_;

  /// \brief
  std::shared_ptr<NearestNeighbors<Motion*> > bakNN_;

  /** \brief The fraction of time the goal is picked as the state to expand
   * towards (if such a state is available) */
  double goalBias_;

  /** \brief The fraction of time the goal is picked as the state to expand
   * towards (if such a state is available) */
  double orphanedBias_;

  /** \brief The maximum length of a motion to be added to a tree */
  double maxDistance_;

  /** \brief The random number generator */
  RNG rng_;

  /** \brief Option to delay and reduce collision checking within iterations
  */
  bool delayCC_;

  /// \brief
  bool dynamicMode_;

  /** \brief Objective we're optimizing */
  base::OptimizationObjectivePtr opt_;

  /** \brief The most recent goal motion.  Used for PlannerData computation
  */
  Motion* lastGoalMotion_;

  /** \brief A list of states in the tree that satisfy the goal condition */
  std::vector<Motion*> goalMotions_;

  /** \brief Stores the Motion containing the last added initial start
  state. */
  Motion* startMotion_;

  /// \brief
  std::vector<ompl::base::State*> orphanedBiasNodes_;
  /// \brief
  std::vector<Motion*> detachedPathNodes_;

  //////////////////////////////
  // Planner progress properties
  /** \brief Number of iterations the algorithm performed */
  unsigned int iterations_;
  /** \brief Best cost found so far by algorithm */
  base::Cost bestCost_;

  /// \brief
  std::size_t maxNodes_;

  /// \brief localPlanning_
  bool localPlanning_;

  /// \brief
  base::State* interimState_;

  /// \brief
  double sampleRadius_;
};

}  // geometric
}  // ompl

#endif
