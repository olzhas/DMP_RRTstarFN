#include "ompl/geometric/DynamicSimpleSetup.h"
#include "ompl/geometric/planners/rrt/DRRTstarFN.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/Exception.h"

namespace ompl {
namespace geometric {

base::DynamicPlannerPtr getDefaultDynamicPlanner(const base::GoalPtr &goal) {
  // TODO create a SelfConfig for Dynamic Planners
  return std::make_shared<ompl::geometric::DRRTstarFN>(
      goal->getSpaceInformation());
}

DynamicSimpleSetup::DynamicSimpleSetup(const base::SpaceInformationPtr &si)
    : configured_(false), lastStatus_(base::PlannerStatus::UNKNOWN) {
  si_ = si;
  pdef_.reset(new base::ProblemDefinition(si_));
}

DynamicSimpleSetup::DynamicSimpleSetup(const base::StateSpacePtr &space)
    : configured_(false), lastStatus_(base::PlannerStatus::UNKNOWN) {
  si_.reset(new base::SpaceInformation(space));
  pdef_.reset(new base::ProblemDefinition(si_));
}

void ompl::geometric::DynamicSimpleSetup::setup() {
  if (!configured_ || !si_->isSetup() || !planner_->isSetup()) {
    if (!si_->isSetup()) si_->setup();
    if (!planner_) {
      if (pa_) planner_ = pa_(si_);
      if (!planner_) {
        OMPL_INFORM("No planner specified. Using default.");

        planner_ = getDefaultDynamicPlanner(getGoal());
      }
    }
    planner_->setProblemDefinition(pdef_);
    if (!planner_->isSetup()) {
      planner_->setup();
    }
    configured_ = true;
  }
}

void DynamicSimpleSetup::readPrecomputedData(std::istream &is) {
  setup();
  clear();
  ompl::base::PlannerData pd(si_);
  ompl::base::PlannerDataStorage pdstorage;
  pdstorage.load(is, pd);
  planner_->setPlannerData(pd);
}

void DynamicSimpleSetup::clear() {
  if (planner_) planner_->clear();
  if (pdef_) pdef_->clearSolutionPaths();
}

bool DynamicSimpleSetup::runSolutionLoop() {
  startLoggerThread();

  setup();

  preplan();

  bool terminate = false;
  auto tNow = std::chrono::high_resolution_clock::now();
  std::size_t i = 0;

  while (!terminate) {
    updateEnvironment();
    if (!validSolution()) {
      pause();
      react();
    }
    terminate = !move();
    // helper function, can be used to log the solution every move the robot
    // does

    if (iterationRoutine_) iterationRoutine_();

    std::this_thread::sleep_until(tNow + timestep_ * (++i));
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

bool DynamicSimpleSetup::validSolution() {
  if (!planner_) {
    OMPL_ERROR("planner is not set, run setup() first");
    std::terminate();
  }

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

  if (pSolutionPath) {
    pathLength_ = pSolutionPath->getStateCount();

    if (step_ < pathLength_ && !completedMotion_) {
      OMPL_INFORM("moving... %d", step_);
      step_++;
      return true;
    }
  }

  OMPL_ERROR("No solution, can't move...");

  return false;
}

base::PlannerStatus DynamicSimpleSetup::plan(double t) {
  ompl::base::PlannerTerminationCondition ptc(
      ompl::base::timedPlannerTerminationCondition(t));

  lastStatus_ = base::PlannerStatus::UNKNOWN;
  time::point start = time::now();
  lastStatus_ = planner_->solve(ptc);
  double planTime_ = time::seconds(time::now() - start);
  if (lastStatus_)
    OMPL_INFORM("Solution found in %f seconds", planTime_);
  else
    OMPL_INFORM("No solution found after %f seconds", planTime_);
  return lastStatus_;
}

base::PlannerStatus DynamicSimpleSetup::plan(
    const base::PlannerTerminationCondition &ptc) {
  lastStatus_ = base::PlannerStatus::UNKNOWN;
  time::point start = time::now();
  lastStatus_ = planner_->solve(ptc);
  double planTime_ = time::seconds(time::now() - start);
  if (lastStatus_)
    OMPL_INFORM("Solution found in %f seconds", planTime_);
  else
    OMPL_INFORM("No solution found after %f seconds", planTime_);
  return lastStatus_;
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

  // TODO rewrite in more generic way
  getDynamicPlanner()->preReact();
  //  std::size_t nodesRemoved =
  //      ss_->getPlanner()->as<DRRTstarFN>()->removeInvalidNodes();
  //    OMPL_INFORM("Nodes removed during clean-up phase: %d", nodesRemoved);
  //
  // ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01125);
  getDynamicPlanner()->react();

  getDynamicPlanner()->postReact();
  // ss_->getProblemDefinition()->clearSolutionPaths();
  // ss_->getPlanner()->as<DRRTstarFN>()->evaluateSolutionPath();

  //    if (!is_reconnected) {
  //      ompl::base::PlannerTerminationCondition ptc(
  //          ompl::base::exactSolnPlannerTerminationCondition(
  //              ss_->getProblemDefinition()));
  //      ss_->solve(ptc);
  //    }

  OMPL_WARN("completed a reaction routine.");
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

  try {
    PathGeometric &p = getSolutionPath();
    p.printAsMatrix(fout);
    fout.close();

    p.interpolate();
    p.printAsMatrix(foutInterp);
    foutInterp.close();

  } catch (ompl::Exception e) {
    OMPL_ERROR(e.what());
  }
}

void DynamicSimpleSetup::recordSolution() {
  if (!haveSolutionPath()) {
    OMPL_INFORM(
        "No solution, there is nothing to record\nWaiting for the one.");
  }
  while (!haveSolutionPath()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  OMPL_INFORM("A solution was found...");

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

void DynamicSimpleSetup::setPlannerData(base::PlannerData &pd) {
  if (planner_) planner_->setPlannerData(pd);
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

void DynamicSimpleSetup::preplan() {
  if (!hasPrecomputedData_) {
    OMPL_INFORM(
        "No precomputed data, preparing the dynamic planner for navigation");

    // here we look for the solution.
    ompl::base::PlannerTerminationCondition ptc(
        ompl::base::exactSolnPlannerTerminationCondition(
            getProblemDefinition()));

    plan(ptc);

    try {
      PathGeometric &p = getSolutionPath();
      p.interpolate();
      pathLength_ = p.getStateCount();
    } catch (ompl::Exception e) {
      OMPL_ERROR("No solution, man!");
      OMPL_ERROR(e.what());
    }

    if (keepComputedData_) {
      std::ofstream solution_file("dynamicsimplesetup.dump");
      saveComputedData(solution_file);
    }
  }
}

std::chrono::milliseconds DynamicSimpleSetup::getTimestep() const
{
    return timestep_;
}

void DynamicSimpleSetup::setTimestep(const std::chrono::milliseconds &timestep)
{
    timestep_ = timestep;
}

}  // geometric
}  // ompl
