#ifndef DYNAMICPLANNER_H
#define DYNAMICPLANNER_H

#include <ompl/base/Planner.h>

namespace ompl {
namespace base {
class DynamicPlanner {
 public:
  // non-copyable
  DynamicPlanner(const DynamicPlanner &) = delete;
  DynamicPlanner &operator=(const DynamicPlanner &) = delete;
  DynamicPlanner(const SpaceInformationPtr &si, const std::string &name);

  virtual ~DynamicPlanner() { ; }

  void prepare();

  void prePause();
  void postPause();
  void preReact();
  void postReact();

  void preMove();
  void postMove();

  PlannerPtr planner_;
};

typedef std::shared_ptr<DynamicPlanner> DynamicPlannerPtr;
}
}

#endif  // DYNAMICPLANNER_H
