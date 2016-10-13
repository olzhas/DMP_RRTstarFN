#ifndef DYNAMICPLANNER_H
#define DYNAMICPLANNER_H

#include "ompl/base/Planner.h"

namespace ompl {
namespace base {

OMPL_CLASS_FORWARD(DynamicPlanner);

class DynamicPlanner : public base::Planner {
 public:
  // non-copyable
  DynamicPlanner(const DynamicPlanner &) = delete;
  DynamicPlanner &operator=(const DynamicPlanner &) = delete;
  DynamicPlanner(const SpaceInformationPtr &si, const std::string &name);

  virtual ~DynamicPlanner() { ; }

  virtual void prepare();
  virtual void prePause();
  virtual void postPause();
  virtual void preReact();
  virtual void postReact();
  virtual void preMove();
  virtual void postMove();
};
}
}

#endif  // DYNAMICPLANNER_H
