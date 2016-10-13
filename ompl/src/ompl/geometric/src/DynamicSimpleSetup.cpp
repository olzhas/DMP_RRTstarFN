#include "ompl/geometric/DynamicSimpleSetup.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/Exception.h"

namespace ompl {
namespace geometric {

void DynamicSimpleSetup::plan() {
  // TODO implement plan()

  OMPL_WARN("void DynamicSimpleSetup::plan()");
}

bool DynamicSimpleSetup::runSolutionLoop() {
  setup();
  prepare();

  bool terminate = false;

  startLoggerThread();

  while (!terminate) {
    auto tNow = std::chrono::high_resolution_clock::now();
    updateEnvironment();
    if (!validSolution()) {
      pause();
      react();
    }
    terminate = !move();
    // helper function, can be used to log the solution every move the robot
    // does

    if (iterationRoutine_) iterationRoutine_();

    std::this_thread::sleep_until(tNow + timestep_);
#ifdef DEBUG
    auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::high_resolution_clock::now() - tNow)
                  .count();
    std::cout << dt << std::endl;
#endif
  }

  stopLoggerThread();

  return true;
}

void DynamicSimpleSetup::prepare() {
  if (!loadPrecomputedData_) {
    ompl::base::PlannerTerminationCondition ptc(
        ompl::base::exactSolnPlannerTerminationCondition(
            getProblemDefinition()));

    // TODO ?
    // solve(ptc);

    // this should be provide by user ?
    auto prepareFn = [&]() {
      double totalTime = getLastPlanComputationTime();
      const double waitPercent = 0.20;
      // solve(waitPercent * totalTime);
    };

    prepareFn();

    try {
      PathGeometric &p = getSolutionPath();
      p.interpolate();
      pathLength_ = p.getStateCount();
    } catch (ompl::Exception e) {
      OMPL_ERROR("No solution, man\n");
    }

    bool saveSolutionFlag = true;
    if (saveSolutionFlag) {
      const std::string solutionFilename("dynamicsimplesetup.dump");
      saveSolution(solutionFilename);
    }

    // prepareUser();
  } else {
    loadPrecomputedPlannerData();
  }

  pSolutionPath = std::make_shared<PathGeometric>(getSolutionPath());
}

bool DynamicSimpleSetup::validSolution() {
  /*
if (planner_ == nullptr) {
  std::cerr << "planner was not initialized\n";
  return false;
}
*/
  if (!validSolutionFn_) {
    OMPL_ERROR("solution validity checker is not defined");
    assert(validSolutionFn_);
    return false;
  }
  if (validSolutionFn_ && !validSolutionFn_()) return false;
  return getProblemDefinition()->hasSolution();
}

bool DynamicSimpleSetup::move() {
  if (pauseMotion_) return true;

  pathLength_ = pSolutionPath->getStateCount();

  if (step_ < pathLength_ && !completedMotion_) {
    OMPL_INFORM("moving... %d", step_);
    step_++;
    return true;
  }

  return false;
}

void DynamicSimpleSetup::pause() {
  pauseMotion_ = true;
  OMPL_INFORM("pausing motion ...");
}

void DynamicSimpleSetup::react() {
  if (!getDynamicPlanner()) {
    OMPL_ERROR("motion planner is not assigned");
    return;
  }
  OMPL_INFORM("initiating a reaction routine...");
  if (getDynamicPlanner() &&
      !getDynamicPlanner()->params().hasParam("dynamic")) {
    OMPL_ERROR("Planner does not contain dynamic parameter");
  }

  getDynamicPlanner()->params().getParam("dynamic")->setValue("true");

  //  // TODO rewrite in more generic way
  //  ss_->getPlanner()->as<DRRTstarFN>()->prepareDynamic(step_);
  //  std::size_t nodesRemoved =
  //      ss_->getPlanner()->as<DRRTstarFN>()->removeInvalidNodes();
  //  OMPL_INFORM("Nodes removed during clean-up phase: %d", nodesRemoved);
  //  //
  //  ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01125);
  //  bool is_reconnected = ss_->getPlanner()->as<DRRTstarFN>()->reconnect();
  //  ss_->getProblemDefinition()->clearSolutionPaths();
  //  ss_->getPlanner()->as<DRRTstarFN>()->evaluateSolutionPath();

  //  if (!is_reconnected) {
  //    ompl::base::PlannerTerminationCondition ptc(
  //        ompl::base::exactSolnPlannerTerminationCondition(
  //            ss_->getProblemDefinition()));
  //    ss_->solve(ptc);
  //  }

  OMPL_WARN("completed a reaction routine.");
}

void DynamicSimpleSetup::loadPrecomputedPlannerData() {
  // TODO loadPrecomputedPlannerData
}

void DynamicSimpleSetup::updateEnvironment() {
  if (updateEnvironmentFn) updateEnvironmentFn();
}

void DynamicSimpleSetup::startLoggerThread() {
  //    logPlannerDataThread_ =
  //    std::thread(std::bind(&DynamicSimpleSetup::recordSolution, this));
  dumpMotionProgress_ = true;
  loggerThread_ = std::thread([this] { recordSolution(); });
}

void DynamicSimpleSetup::stopLoggerThread() {
  dumpMotionProgress_ = false;
  loggerThread_.join();
}

void DynamicSimpleSetup::saveSolution(const std::string &fname) {
  // Get the planner data to visualize the vertices and the edges
  ompl::base::PlannerData pdat(getSpaceInformation());
  getPlannerData(pdat);

  ompl::base::PlannerDataStorage pdstorage;

  pdstorage.store(pdat, fname.c_str());

  // FIX code duplicate ???
  OMPL_WARN("saving solution information...");

  if (!haveSolutionPath()) {
    OMPL_ERROR("No solution, there is nothing to record");
    return;
  }

  std::string filename("dubins-results");
  std::string filenameSolution("dubins-results-is_solution");

  std::ofstream fout;
  std::ofstream foutInterp;
  std::ofstream foutSolution;

  fout.open(filename + ".dat");
  foutInterp.open(filename + "-interp.dat");
  foutSolution.open(filenameSolution + ".dat");

  foutSolution << haveExactSolutionPath() << std::endl;
  foutSolution.close();

  PathGeometric &p = getSolutionPath();
  p.printAsMatrix(fout);
  fout.close();

  p.interpolate();
  p.printAsMatrix(foutInterp);
  foutInterp.close();
}

void DynamicSimpleSetup::recordSolution() {
  if (!haveSolutionPath()) {
    OMPL_ERROR("No solution, there is nothing to record");
    return;
  }

  std::string filename("dubins-results");
  std::string filenameSolution("dubins-results-is_solution");

  int dumpNumber = 0;

  while (dumpMotionProgress_) {
    std::ofstream fout;
    std::ofstream foutInterp;
    std::ofstream foutSolution;

    fout.open(filename + std::to_string(dumpNumber) + ".dat");
    foutInterp.open(filename + std::to_string(dumpNumber) + "-interp.dat");
    foutSolution.open(filenameSolution + std::to_string(dumpNumber) + ".dat");

    foutSolution << haveExactSolutionPath() << std::endl;
    foutSolution.close();

    PathGeometric &p = getSolutionPath();
    p.printAsMatrix(fout);
    fout.close();

    p.interpolate();
    p.printAsMatrix(foutInterp);
    foutInterp.close();

    ++dumpNumber;
    auto waitDuration = std::chrono::milliseconds(100);
    std::this_thread::sleep_for(waitDuration);
  }
}

void DynamicSimpleSetup::setSolutionValidityFunction(
    std::function<bool(void)> &fn) {
  validSolutionFn_ = fn;
}

void DynamicSimpleSetup::setIterationRoutine(std::function<bool(void)> &fn) {
  iterationRoutine_ = fn;
}

base::PlannerPtr getDefaultPlanner(const base::GoalPtr &goal) {
  return tools::SelfConfig::getDefaultPlanner(goal);
}

DynamicSimpleSetup::DynamicSimpleSetup(const base::SpaceInformationPtr &si)
    : configured_(false),
      planTime_(0.0),
      simplifyTime_(0.0),
      lastStatus_(base::PlannerStatus::UNKNOWN) {
  si_ = si;
  pdef_.reset(new base::ProblemDefinition(si_));
}

DynamicSimpleSetup::DynamicSimpleSetup(const base::StateSpacePtr &space)
    : configured_(false),
      planTime_(0.0),
      simplifyTime_(0.0),
      lastStatus_(base::PlannerStatus::UNKNOWN) {
  si_.reset(new base::SpaceInformation(space));
  pdef_.reset(new base::ProblemDefinition(si_));
}

void ompl::geometric::DynamicSimpleSetup::setup() {
  if (!configured_ || !si_->isSetup() || !planner_->isSetup()) {
    if (!si_->isSetup()) si_->setup();
    if (!planner_) {
      // FIXME
      // if (pa_) planner_ = pa_(si_);
      if (!planner_) {
        OMPL_INFORM("No planner specified. Using default.");
        // TODO
        // planner_ = tools::SelfConfig::getDefaultPlanner(getGoal());
      }
    }
    planner_->setProblemDefinition(pdef_);
    if (!planner_->isSetup()) planner_->setup();
    configured_ = true;
  }
}

void DynamicSimpleSetup::clear() {
  if (planner_) planner_->clear();
  if (pdef_) pdef_->clearSolutionPaths();
}

void DynamicSimpleSetup::setStartAndGoalStates(const base::ScopedState<> &start,
                                               const base::ScopedState<> &goal,
                                               const double threshold) {
  pdef_->setStartAndGoalStates(start, goal, threshold);

  // Clear any past solutions since they no longer correspond to our start and
  // goal states
  pdef_->clearSolutionPaths();

  psk_.reset(new PathSimplifier(si_, pdef_->getGoal()));
}

void DynamicSimpleSetup::setGoalState(const base::ScopedState<> &goal,
                                      const double threshold) {
  pdef_->setGoalState(goal, threshold);
  psk_.reset(new PathSimplifier(si_, pdef_->getGoal()));
}

/** \brief Set the goal for planning. This call is not
    needed if setStartAndGoalStates() has been called. */
void DynamicSimpleSetup::setGoal(const base::GoalPtr &goal) {
  pdef_->setGoal(goal);

  if (goal && goal->hasType(base::GOAL_SAMPLEABLE_REGION))
    psk_.reset(new PathSimplifier(si_, pdef_->getGoal()));
  else
    psk_.reset(new PathSimplifier(si_));
}

//// we provide a duplicate implementation here to allow the planner to choose
/// how
//// the time is turned into a planner termination condition
// base::PlannerStatus DynamicSimpleSetup::solve(double time) {
//  setup();
//  lastStatus_ = base::PlannerStatus::UNKNOWN;
//  time::point start = time::now();
//  lastStatus_ = planner_->solve(time);
//  planTime_ = time::seconds(time::now() - start);
//  if (lastStatus_)
//    OMPL_INFORM("Solution found in %f seconds", planTime_);
//  else
//    OMPL_INFORM("No solution found after %f seconds", planTime_);
//  return lastStatus_;
//}

// ompl::base::PlannerStatus DynamicSimpleSetup::solve(
//    const base::PlannerTerminationCondition &ptc) {
//  setup();
//  lastStatus_ = base::PlannerStatus::UNKNOWN;
//  time::point start = time::now();
//  lastStatus_ = planner_->solve(ptc);
//  planTime_ = time::seconds(time::now() - start);
//  if (lastStatus_)
//    OMPL_INFORM("Solution found in %f seconds", planTime_);
//  else
//    OMPL_INFORM("No solution found after %f seconds", planTime_);
//  return lastStatus_;
//}

void DynamicSimpleSetup::simplifySolution(
    const base::PlannerTerminationCondition &ptc) {
  if (pdef_) {
    const base::PathPtr &p = pdef_->getSolutionPath();
    if (p) {
      time::point start = time::now();
      PathGeometric &path = static_cast<PathGeometric &>(*p);
      std::size_t numStates = path.getStateCount();
      psk_->simplify(path, ptc);
      simplifyTime_ = time::seconds(time::now() - start);
      OMPL_INFORM(
          "SimpleSetup: Path simplification took %f seconds and changed from "
          "%d to %d states",
          simplifyTime_, numStates, path.getStateCount());
      return;
    }
  }
  OMPL_WARN("No solution to simplify");
}

void DynamicSimpleSetup::simplifySolution(double duration) {
  if (pdef_) {
    const base::PathPtr &p = pdef_->getSolutionPath();
    if (p) {
      time::point start = time::now();
      PathGeometric &path = static_cast<PathGeometric &>(*p);
      std::size_t numStates = path.getStateCount();
      if (duration < std::numeric_limits<double>::epsilon())
        psk_->simplifyMax(static_cast<PathGeometric &>(*p));
      else
        psk_->simplify(static_cast<PathGeometric &>(*p), duration);
      simplifyTime_ = time::seconds(time::now() - start);
      OMPL_INFORM(
          "SimpleSetup: Path simplification took %f seconds and changed from "
          "%d to %d states",
          simplifyTime_, numStates, path.getStateCount());
      return;
    }
  }
  OMPL_WARN("No solution to simplify");
}

const std::string DynamicSimpleSetup::getSolutionPlannerName(void) const {
  if (pdef_) {
    const ompl::base::PathPtr path;  // convert to a generic path ptr
    ompl::base::PlannerSolution solution(path);  // a dummy solution

    // Get our desired solution
    pdef_->getSolution(solution);
    return solution.plannerName_;
  }
  throw Exception("No problem definition found");
}

ompl::geometric::PathGeometric &DynamicSimpleSetup::getSolutionPath() const {
  if (pdef_) {
    const base::PathPtr &p = pdef_->getSolutionPath();
    if (p) return static_cast<PathGeometric &>(*p);
  }
  throw Exception("No solution path");
}

bool DynamicSimpleSetup::haveExactSolutionPath() const {
  return haveSolutionPath() && (!pdef_->hasApproximateSolution() ||
                                pdef_->getSolutionDifference() <
                                    std::numeric_limits<double>::epsilon());
}

void DynamicSimpleSetup::getPlannerData(base::PlannerData &pd) const {
  pd.clear();
  if (planner_) planner_->getPlannerData(pd);
}

void DynamicSimpleSetup::print(std::ostream &out) const {
  if (si_) {
    si_->printProperties(out);
    si_->printSettings(out);
  }
  if (planner_) {
    planner_->printProperties(out);
    planner_->printSettings(out);
  }
  if (pdef_) pdef_->print(out);
}
}
}
