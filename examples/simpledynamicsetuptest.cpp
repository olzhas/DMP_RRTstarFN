#include <Eigen/Eigen>
#include <dart/common/common.h>

#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/config.h>
#include <ompl/util/Exception.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <fstream>
#include <functional>
#include <iostream>
#include <memory>

// TODO fix this ugly inclusion with cmake
#include "../ompl/geometric/planners/rrt/DRRTstarFN.h"
#include "../ompl/geometric/DynamicSimpleSetup.h"
#include "../model/model.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

const std::string placeholder = "data/obstacles/test.map";

class DubinsCarEnvironment {
 public:
  DubinsCarEnvironment()
      : maxWidth_(1000), maxHeight_(1000), pModel_(new Model(placeholder)) {
    ob::StateSpacePtr space(new ob::DubinsStateSpace);
    //   if(dss_.use_count() == 0)
    //    delete dss_.get();
    dss_.reset(new og::DynamicSimpleSetup(space));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.high[0] = maxWidth_;
    bounds.high[1] = maxHeight_;

    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // set state validity checking for this space
    ob::SpaceInformationPtr spaceInfo = dss_->getSpaceInformation();
    pModel_->setSpaceInformation(dss_->getSpaceInformation());
    dss_->setStateValidityChecker(
        std::bind(&Model::isStateValid, pModel_, std::placeholders::_1));
    space->setup();
    dss_->setPlanner(ob::PlannerPtr(new og::DRRTstarFN(spaceInfo)));
    dss_->getPlanner()->as<og::DRRTstarFN>()->setRange(35.0);
    dss_->getPlanner()->as<og::DRRTstarFN>()->setMaxNodes(15000);
    dss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01125);


    ob::ScopedState<> start(dss_->getStateSpace());
    start[0] = 1;
    start[1] = 1;
    ob::ScopedState<> goal(dss_->getStateSpace());
    goal[0] = 100;
    goal[1] = 100;
    dss_->setStartAndGoalStates(start, goal);
    // generate a few solutions; all will be added to the goal;

    dss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(0.0015);

  }

  void execute() {
    if (dss_->drive()) {
      std::cout << "Success" << std::endl;
    }
  }

 private:
  og::DynamicSimpleSetupPtr dss_;
  const double maxWidth_;
  const double maxHeight_;

  std::string prefix_;

  std::shared_ptr<Model> pModel_;
};

int main() {
  DubinsCarEnvironment env;

  env.execute();

  return 0;
}
