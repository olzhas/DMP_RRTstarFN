#ifndef DYNAMICSIMPLESETUP_H
#define DYNAMICSIMPLESETUP_H

#include "ompl/base/DynamicPlanner.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerDataStorage.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/util/Console.h"
#include "ompl/util/Deprecation.h"
#include "ompl/util/Exception.h"

#include <chrono>
#include <thread>

namespace ompl {
namespace geometric {

OMPL_CLASS_FORWARD(DynamicSimpleSetup);

/**
 * \brief The DynamicSimpleSetup class
 * We assume that robot perfectly follows the trajectory
 */
class DynamicSimpleSetup {
 public:
  /** \brief Constructor needs the state space used for planning. */
  explicit DynamicSimpleSetup(const base::SpaceInformationPtr &si);

  /** \brief Constructor needs the state space used for planning. */
  explicit DynamicSimpleSetup(const base::StateSpacePtr &space);

  /// \brief Destructor
  virtual ~DynamicSimpleSetup() { ; }

  /** \brief Get the current instance of the space information */
  const base::SpaceInformationPtr &getSpaceInformation() const { return si_; }

  /** \brief Get the current instance of the problem definition */
  const base::ProblemDefinitionPtr &getProblemDefinition() const {
    return pdef_;
  }

  /** \brief Get the current instance of the state space */
  const base::StateSpacePtr &getStateSpace() const {
    return si_->getStateSpace();
  }

  /** \brief Get the current instance of the state validity checker */
  const base::StateValidityCheckerPtr &getStateValidityChecker() const {
    return si_->getStateValidityChecker();
  }

  /** \brief Get the current goal definition */
  const base::GoalPtr &getGoal() const { return pdef_->getGoal(); }

  /** \brief Get the current planner */
  const base::DynamicPlannerPtr &getDynamicPlanner() const { return planner_; }

  /** \brief Get the planner allocator */
  const base::DynamicPlannerAllocator &getPlannerAllocator() const {
    return pa_;
  }

  /** \brief Get the path simplifier */
  const PathSimplifierPtr &getPathSimplifier() const { return psk_; }

  /** \brief Get the path simplifier */
  PathSimplifierPtr &getPathSimplifier() { return psk_; }

  /** \brief Get the optimization objective to use */
  const base::OptimizationObjectivePtr &getOptimizationObjective() const {
    return pdef_->getOptimizationObjective();
  }

  /** \brief Return true if a solution path is available (previous call to
   * solve() was successful) and the solution is exact (not approximate) */
  bool haveExactSolutionPath() const;

  /** \brief Return true if a solution path is available (previous call to
   * solve() was successful). The solution may be approximate. */
  bool haveSolutionPath() const { return pdef_->getSolutionPath().get(); }

  /** \brief Get the best solution's planer name. Throw an exception if no
   * solution is available */
  const std::string getSolutionPlannerName(void) const;

  /** \brief Get the solution path. Throw an exception if no solution is
   * available */
  PathGeometric &getSolutionPath() const;

  /** \brief Get information about the exploration data structure the motion
   * planner used. */
  void getPlannerData(base::PlannerData &pd) const;

  /** \brief Set information about the exploration data structure the motion
   * planner used. */
  void setPlannerData(base::PlannerData &pd);

  /** \brief Set the state validity checker to use */
  void setStateValidityChecker(const base::StateValidityCheckerPtr &svc) {
    si_->setStateValidityChecker(svc);
  }

  /** \brief Set the state validity checker to use */
  void setStateValidityChecker(const base::StateValidityCheckerFn &svc) {
    si_->setStateValidityChecker(svc);
  }

  /** \brief Set the optimization objective to use */
  void setOptimizationObjective(
      const base::OptimizationObjectivePtr &optimizationObjective) {
    pdef_->setOptimizationObjective(optimizationObjective);
  }

  /** \brief Set the start and goal states to use. */
  void setStartAndGoalStates(
      const base::ScopedState<> &start, const base::ScopedState<> &goal,
      const double threshold = std::numeric_limits<double>::epsilon());

  /** \brief Add a starting state for planning. This call is not
      needed if setStartAndGoalStates() has been called. */
  void addStartState(const base::ScopedState<> &state) {
    pdef_->addStartState(state);
  }

  /** \brief Clear the currently set starting states */
  void clearStartStates() { pdef_->clearStartStates(); }

  /** \brief Clear the currently set starting states and add \e state as the
   * starting state */
  void setStartState(const base::ScopedState<> &state) {
    clearStartStates();
    addStartState(state);
  }

  /** \brief A simple form of setGoal(). The goal will be an instance of
   * ompl::base::GoalState */
  void setGoalState(
      const base::ScopedState<> &goal,
      const double threshold = std::numeric_limits<double>::epsilon());

  /** \brief Set the goal for planning. This call is not
      needed if setStartAndGoalStates() has been called. */
  void setGoal(const base::GoalPtr &goal);

  /** \brief Set the planner to use. If the planner is not
      set, an attempt is made to use the planner
      allocator. If no planner allocator is available
      either, a default planner is set. */
  void setDynamicPlanner(const base::DynamicPlannerPtr &planner) {
    if (planner && planner->getSpaceInformation().get() != si_.get())
      throw Exception("Planner instance does not match space information");
    planner_ = planner;
    configured_ = false;
  }

  /** \brief Set the planner allocator to use. This is only
      used if no planner has been set. This is optional -- a default
      planner will be used if no planner is otherwise specified. */
  void setPlannerAllocator(const base::DynamicPlannerAllocator &pa) {
    pa_ = pa;
    planner_.reset();
    configured_ = false;
  }

  /** \brief Return the status of the last planning attempt */
  base::PlannerStatus getLastPlannerStatus() const { return lastStatus_; }

  /** \brief Attempt to simplify the current solution path. Spent at most \e
     duration seconds in the simplification process.
      If \e duration is 0 (the default), a default simplification procedure is
     executed. */
  void simplifySolution(double duration = 0.0);

  /** \brief Attempt to simplify the current solution path. Stop computation
   * when \e ptc becomes true at the latest. */
  void simplifySolution(const base::PlannerTerminationCondition &ptc);

  /** \brief Clear all planning data. This only includes
      data generated by motion plan computation. Planner
      settings, start & goal states are not affected. */
  virtual void clear();

  /** \brief Print information about the current setup */
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief This method will create the necessary classes
      for planning. The solve() method will call this
      function automatically. */
  virtual void setup();

  ///
  /// \brief saveSolution
  ///
  void saveSolution(const std::string &);

  /** \brief this method reads data from the stream */

  void readPrecomputedData(std::istream &is);

  /** \brief this method makes */

  void enableKeepComputedData() { keepComputedData_ = true; }

  /** \brief this method makes */
  void disableKeepComputedData() { keepComputedData_ = false; }

  /** \brief this method implements the solution loop with reactive planning */
  bool runSolutionLoop();

  /// \brief prepareUser
  std::function<void()> prepareUser;
  /// \brief reactUser
  std::function<void()> reactUser;
  /// \brief clearUser
  std::function<void()> clearUser;
  /// \brief updateEnvironmentFn
  std::function<void()> updateEnvironmentFn;

  ///
  /// \brief setSolutionValidityFunction
  /// \param fn
  ///
  void setSolutionValidityFunction(std::function<bool(void)> &fn);

  ///
  /// \brief setIterationRoutine
  /// \param fn
  ///
  void setIterationRoutine(std::function<bool(void)> &fn);

  ///
  /// \brief loadPrecomputedData
  /// \param is
  ///
  void loadPrecomputedData(std::istream &is) {
    ompl::base::PlannerData pd(si_);
    ompl::base::PlannerDataStorage pdstorage;
    pdstorage.load(is, pd);
    hasPrecomputedData_ = true;
    setPlannerData(pd);
  }

  ///
  /// \brief saveComputedData
  /// \param os
  ///
  void saveComputedData(std::ostream &os) {
    // Get the planner data to visualize the vertices and the edges
    ompl::base::PlannerData pd(si_);
    getPlannerData(pd);
    ompl::base::PlannerDataStorage pdstorage;
    pdstorage.store(pd, os);
  }

  /**
   * \brief this method does preparation for the dynamic motion planning this
   * can include heavily computational tasks
   */
  void preplan();

  ///
  /// \brief getTimestep
  /// \return
  ///
  std::chrono::milliseconds getTimestep() const;

  ///
  /// \brief setTimestep
  /// \param timestep
  ///
  void setTimestep(const std::chrono::milliseconds &timestep);

  ///
  /// \brief getUpdateEnvironmentFn
  /// \return
  ///
  std::function<void()> getUpdateEnvironmentFn() const;

  ///
  /// \brief setUpdateEnvironmentFn
  /// \param value
  ///
  void setUpdateEnvironmentFn(const std::function<void()> &value);

 private:
  /** \brief time step between regular obstacle collision routine in
   * milliseconds */

  ompl::base::DynamicPlannerPtr dynamicPlanner_ = nullptr;

  ///
  /// \brief timestep_
  ///
  std::chrono::milliseconds timestep_ = std::chrono::milliseconds(30);  // dt

  /** \brief reaction routine */
  void react();

  /** \brief work on problem */
  base::PlannerStatus plan(double t = 1.0);

  /** \brief work on problem */
  base::PlannerStatus plan(const base::PlannerTerminationCondition &ptc);

  /** \brief increment motion */
  bool move();

  /** \brief update environment informatin */
  void updateEnvironment();

  /** \brief check solution for validity includes environment information update
   * routine */
  bool validSolution();

  /** \brief pause robot motion */
  void pause();

  /** \brief */
  void recordSolution();

  /** \brief start the logging thread,
   *
   * TODO where should it start ? */
  void startLoggerThread();

  /** \brief stop the logging thread */
  void stopLoggerThread();

  /** \brief load precomputed data from file ?*/
  bool hasPrecomputedData_ = false;

  /**
   * \brief keepPrecomputedData_
   */
  bool keepComputedData_ = false;

  /** \brief motion termination flag */
  bool completedMotion_ = false;

  /** \brief motion pause flag */
  bool pauseMotion_ = false;

  /** \brief current step */
  std::size_t step_ = 0;

  /** \brief length of the solution path */
  std::size_t pathLength_ = 0;

  /** \brief dump preparation phase data to file */
  bool dumpPrepareInfo_ = false;

  /** \brief dump motion progress phase data to file */
  bool dumpMotionProgress_ = false;

  ///
  std::thread loggerThread_;

  ///
  std::shared_ptr<ompl::geometric::PathGeometric> pSolutionPath;

  /// solution validity checker function
  std::function<bool(void)> validSolutionFn_;

  ///
  std::function<void(void)> iterationRoutine_;

 protected:
  /// The created space information
  base::SpaceInformationPtr si_;

  /// The created problem definition
  base::ProblemDefinitionPtr pdef_;

  /// The maintained planner instance
  base::DynamicPlannerPtr planner_;

  /// The optional planner allocator
  base::DynamicPlannerAllocator pa_;

  /// The instance of the path simplifier
  PathSimplifierPtr psk_;

  /// Flag indicating whether the classes needed for planning are set up
  bool configured_;

  /// The status of the last planning request
  base::PlannerStatus lastStatus_;
};
}
}

#endif  // DYNAMICMOTIONPLANNINGSETUP_H
