#include <chrono>
#include <thread>

#include "../DynamicSimpleSetup.h"

namespace ompl {
namespace geometric {

DynamicSimpleSetup::DynamicSimpleSetup(ompl::base::SpaceInformationPtr &si)
    : ompl::geometric::SimpleSetup(si) {}

DynamicSimpleSetup::DynamicSimpleSetup(ompl::base::StateSpacePtr &space)
    : ompl::geometric::SimpleSetup(space) {}

void DynamicSimpleSetup::setup() {
  SimpleSetup::setup();
  timestep = std::chrono::milliseconds(100);
}

void DynamicSimpleSetup::clear() {}

void DynamicSimpleSetup::plan() {}

bool DynamicSimpleSetup::drive() {
  setup();
  prepare();

  bool terminate = false;
  while (!terminate) {
    std::chrono::high_resolution_clock::time_point tNow =
        std::chrono::high_resolution_clock::now();
    move();
    if (!validSolution()) {
      stop();
      react();
    }
    std::this_thread::sleep_until(tNow + timestep);
#ifdef DEBUG
    auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::high_resolution_clock::now() - tNow).count();
    std::cout << dt << std::endl;
#endif
  }
  return true;
}

void DynamicSimpleSetup::prepare() {
  if (!loadPrecomputedData_) {
    ompl::base::PlannerTerminationCondition ptc(
        ompl::base::exactSolnPlannerTerminationCondition(
            getProblemDefinition()));

    solve(ptc);
  }
  // prepareUser();
}

bool DynamicSimpleSetup::validSolution() {
  if (planner_ == nullptr) {
    std::cerr << "planner was not initialized\n";
    return false;
  }

  // if (planner_->checkValidity()) return false;

  return true;
}

void DynamicSimpleSetup::move() {}

void DynamicSimpleSetup::stop() {}

void DynamicSimpleSetup::react() {}

void DynamicSimpleSetup::loadPrecomputed() { /*planner_->load(); */
}
}
}

