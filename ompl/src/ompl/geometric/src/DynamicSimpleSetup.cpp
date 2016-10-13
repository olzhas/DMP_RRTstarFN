#include "ompl/geometric/DynamicSimpleSetup.h"
#include <ompl/util/Exception.h>
#include "ompl/geometric/planners/rrt/DRRTstarFN.h"

namespace ompl {
namespace geometric {

DynamicSimpleSetup::DynamicSimpleSetup(ompl::base::SpaceInformationPtr &si) {
  ss_.reset(new ompl::geometric::SimpleSetup(si));
}

DynamicSimpleSetup::DynamicSimpleSetup(ompl::base::StateSpacePtr &space) {
  ss_.reset(new ompl::geometric::SimpleSetup(space));
}

void DynamicSimpleSetup::setup() {
  ss_->setup();
  timestep = std::chrono::milliseconds(30);
}

void DynamicSimpleSetup::clear() {
  // TODO extend clear()
  ss_->clear();
  OMPL_WARN("void DynamicSimpleSetup::clear()");
}

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

    std::this_thread::sleep_until(tNow + timestep);
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
            ss_->getProblemDefinition()));

    ss_->solve(ptc);

    // this should be provide by user ?
    auto prepareFn = [&]() {
      double totalTime = ss_->getLastPlanComputationTime();
      const double waitPercent = 0.20;
      ss_->solve(waitPercent * totalTime);
    };

    prepareFn();

    try {
      PathGeometric &p = ss_->getSolutionPath();
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

  pSolutionPath = std::make_shared<PathGeometric>(ss_->getSolutionPath());
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
  return ss_->getProblemDefinition()->hasSolution();
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
  if (!ss_->getPlanner()) {
    OMPL_ERROR("motion planner is not assigned");
    return;
  }
  OMPL_INFORM("initiating a reaction routine...");
  if (ss_->getPlanner() && !ss_->getPlanner()->params().hasParam("dynamic")) {
    OMPL_ERROR("Planner does not contain dynamic parameter");
  }

  ss_->getPlanner()->params().getParam("dynamic")->setValue("true");

//  // TODO rewrite in more generic way
//  ss_->getPlanner()->as<DRRTstarFN>()->prepareDynamic(step_);
//  std::size_t nodesRemoved =
//      ss_->getPlanner()->as<DRRTstarFN>()->removeInvalidNodes();
//  OMPL_INFORM("Nodes removed during clean-up phase: %d", nodesRemoved);
//  // ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01125);
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
  ompl::base::PlannerData pdat(ss_->getSpaceInformation());
  ss_->getPlannerData(pdat);

  ompl::base::PlannerDataStorage pdstorage;

  pdstorage.store(pdat, fname.c_str());

  // FIX code duplicate ???
  OMPL_WARN("saving solution information...");

  if (!ss_->haveSolutionPath()) {
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

  foutSolution << ss_->haveExactSolutionPath() << std::endl;
  foutSolution.close();

  PathGeometric &p = ss_->getSolutionPath();
  p.printAsMatrix(fout);
  fout.close();

  p.interpolate();
  p.printAsMatrix(foutInterp);
  foutInterp.close();
}

void DynamicSimpleSetup::recordSolution() {
  if (!ss_->haveSolutionPath()) {
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

    foutSolution << ss_->haveExactSolutionPath() << std::endl;
    foutSolution.close();

    PathGeometric &p = ss_->getSolutionPath();
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
}
}
