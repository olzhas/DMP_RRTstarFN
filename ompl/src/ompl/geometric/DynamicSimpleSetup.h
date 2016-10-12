#ifndef DYNAMICSIMPLESETUP_H
#define DYNAMICSIMPLESETUP_H

#include <chrono>
#include <fstream>
#include <thread>

#include <ompl/base/PlannerDataStorage.h>
#include <ompl/geometric/SimpleSetup.h>
#include "ompl/base/DynamicPlanner.h"

namespace ompl {
namespace geometric {

class DynamicSimpleSetup {
 public:
  explicit DynamicSimpleSetup(ompl::base::SpaceInformationPtr& si);

  explicit DynamicSimpleSetup(ompl::base::StateSpacePtr& space);

  ~DynamicSimpleSetup() { ; }

  /** \brief */
  void saveSolution(const std::string&);

  /** \brief */
  void loadPrecomputedPlannerData();

  /** \brief drive the robot */
  bool runSolutionLoop();

  /** \brief user supplied prepeartion routine*/
  std::function<void()> prepareUser;
  std::function<void()> reactUser;
  std::function<void()> clearUser;
  std::function<void()> updateEnvironmentFn;

  void setSolutionValidityFunction(std::function<bool(void)>& fn);
  void setIterationRoutine(std::function<bool(void)>& fn);

  base::PlannerPtr getPlanner() const { return ss_->getPlanner(); }

  base::SpaceInformationPtr getSpaceInformation() const {
    return ss_->getSpaceInformation();
  }

  base::StateSpacePtr getStateSpace() const { return ss_->getStateSpace(); }

  void setStartAndGoalStates(const base::ScopedState<>& start,
                             const base::ScopedState<>& goal) {
    ss_->setStartAndGoalStates(start, goal);
  }

  void setPlanner(const base::PlannerPtr& planner) { ss_->setPlanner(planner); }

  void setStateValidityChecker(const base::StateValidityCheckerFn& svc) {
    ss_->setStateValidityChecker(svc);
  }

 private:
  /** \brief time step between regular obstacle collision routine in
   * milliseconds */

  ompl::base::DynamicPlannerPtr planner_ = nullptr;

  std::chrono::milliseconds timestep;  // dt

  /** \brief call back for obstacle collision routine */
  bool (*collisionChecker)();

  /** \brief reaction routine */
  void react();

  /** \brief reset to default value of the algorithm */
  void setup();

  void clear();

  /** \brief work on problem */
  void plan();

  /** \brief preparation step for the dynamic motion planning */
  void prepare();

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

  /** \brief start the logging thread, TODO where should it start ? */
  void startLoggerThread();

  /** \brief stop the logging thread */
  void stopLoggerThread();

  /** \brief load precomputed data from file ?*/
  bool loadPrecomputedData_ = false;

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

  std::thread loggerThread_;

  std::shared_ptr<ompl::geometric::PathGeometric> pSolutionPath;

  std::function<bool(void)> validSolutionFn_;

  std::function<void(void)> iterationRoutine_;

  ///
  /// \brief ss_
  ///
  ompl::geometric::SimpleSetupPtr ss_;
};

typedef std::shared_ptr<DynamicSimpleSetup> DynamicSimpleSetupPtr;
}
}

#endif  // DYNAMICMOTIONPLANNINGSETUP_H
