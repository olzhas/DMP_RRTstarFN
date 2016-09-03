#include "ompl/base/DynamicPlanner.h"

namespace ompl {
namespace base {

DynamicPlanner::DynamicPlanner(const SpaceInformationPtr &si,
                               const std::string &name)
    : Planner(si, name) {
  if (!si_)
    throw Exception(name_, "Invalid space information instance for planner");
}
}
}
