#include "ompl/base/DynamicPlanner.h"

namespace ompl {
namespace base {

DynamicPlanner::DynamicPlanner(const SpaceInformationPtr &si,
                               const std::string &name) {
  planner_ = std::make_shared<Planner>(si, name);
  if (!planner_->getSpaceInformation())
    throw Exception(planner_->getName(),
                    "Invalid space information instance for planner");
}
}
}
