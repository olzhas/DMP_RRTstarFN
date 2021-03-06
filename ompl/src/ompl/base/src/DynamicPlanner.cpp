#include "ompl/base/DynamicPlanner.h"

namespace ompl {
namespace base {

DynamicPlanner::DynamicPlanner(const SpaceInformationPtr &si,
                               const std::string &name)
    : Planner(si, name) {
  if (!si_)
    throw Exception(name_, "Invalid space information instance for planner");
}

void DynamicPlanner::prepare() { ; }
void DynamicPlanner::prePause() { ; }
void DynamicPlanner::postPause() { ; }
void DynamicPlanner::preReact() { ; }
void DynamicPlanner::postReact() { ; }
void DynamicPlanner::preMove() { ; }
void DynamicPlanner::postMove() { ; }
void DynamicPlanner::react() { ; }

void DynamicPlanner::setPlannerData(const ompl::base::PlannerData &data) {
  pdef_->clearStartStates();
  pdef_->clearGoal();
  pdef_->clearSolutionPaths();
}

}  // base
}  // ompl
