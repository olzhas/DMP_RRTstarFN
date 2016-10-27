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

  /**
   * @brief prepare
   */
  virtual void prepare();
  /**
   * @brief react
   */
  virtual void react();
  /**
   * @brief prePause
   */
  virtual void prePause();
  /**
   * @brief postPause
   */
  virtual void postPause();
  /**
   * @brief preReact
   */
  virtual void preReact();
  /**
   * @brief postReact
   */
  virtual void postReact();
  /**
   * @brief preMove
   */
  virtual void preMove();
  /**
   * @brief postMove
   */
  virtual void postMove();

  /**
   * @brief setPlannerData
   * @param data
   */
  virtual void setPlannerData(const ompl::base::PlannerData &data);
};

typedef std::function<base::DynamicPlannerPtr(
    const base::SpaceInformationPtr &)>
    DynamicPlannerAllocator;
}
}

#endif  // DYNAMICPLANNER_H
