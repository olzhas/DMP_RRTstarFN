#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/config.h>
#include <ompl/util/Exception.h>

#include <fstream>
#include <functional>
#include <iostream>
#include <memory>

#include "model.h"
#include "ompl/geometric/DynamicSimpleSetup.h"
#include "ompl/geometric/planners/rrt/DRRTstarFN.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

constexpr double kMaxHeight = 1000;
constexpr double kMaxWidth = 1000;
constexpr double minimumRadius = 5;

constexpr char kPrecomputedDataFilename[] = "dynamicsimplesetup.dump";
const std::string kPlaceholder = "data/obstacles/test.map";
const std::string kDynamicInformationFile = "data/obstacles/dynamic_first.txt";

class DubinsCarEnvironment {
 public:
  DubinsCarEnvironment()
      : maxWidth_(kMaxWidth),
        maxHeight_(kMaxHeight),
        pModel_(new Model(kPlaceholder)) {
    ob::StateSpacePtr space(new ob::DubinsStateSpace(minimumRadius));

    dss_.reset(new og::DynamicSimpleSetup(space));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.high[0] = maxWidth_;
    bounds.high[1] = maxHeight_;

    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // set state validity checking for this space
    const ob::SpaceInformationPtr& si = dss_->getSpaceInformation();
    pModel_->setSpaceInformation(si);
    dss_->setStateValidityChecker(
        std::bind(&Model::isStateValid, pModel_, std::placeholders::_1));
    space->setup();

    auto planner = std::make_shared<og::DRRTstarFN>(si);
    planner->setRange(35.0);
    planner->setMaxNodes(15000);
    si->setStateValidityCheckingResolution(0.01125);
    dss_->setDynamicPlanner(ob::DynamicPlannerPtr(planner));

    planner->setGoalBias(0.0015);

    bool failedToLoad = false;
    if (hasPrecomputedData_) {
      std::ifstream precompDataFileStream(kPrecomputedDataFilename);
      if (precompDataFileStream) {
        dss_->readPrecomputedData(precompDataFileStream);
      } else {
        failedToLoad = true;
      }
    }
    if (failedToLoad) {
      ob::ScopedState<> start(dss_->getStateSpace());
      start[0] = 80;
      start[1] = 80;
      ob::ScopedState<> goal(dss_->getStateSpace());
      goal[0] = 700;
      goal[1] = 700;
      dss_->setStartAndGoalStates(start, goal);
    }

    std::function<bool(void)> dummyLambda = []() -> bool { return true; };
    dss_->setSolutionValidityFunction(dummyLambda);
    dss_->enableKeepComputedData();
    dss_->setUpdateEnvironmentFn(std::bind(&Model::updateObstacles, pModel_));
  }

  void launch() {
    std::cout << "running problem" << std::endl;
    if (dss_->runSolutionLoop()) {
      std::cout << "Success" << std::endl;
    }
  }

 private:
  og::DynamicSimpleSetupPtr dss_;
  const double maxWidth_;
  const double maxHeight_;

  std::string prefix_;
  std::shared_ptr<Model> pModel_;

  bool hasPrecomputedData_ = true;
};

int main() {
  DubinsCarEnvironment env;
  env.launch();

  // this will be reached iff robot reached the goal state

  return EXIT_SUCCESS;
}
