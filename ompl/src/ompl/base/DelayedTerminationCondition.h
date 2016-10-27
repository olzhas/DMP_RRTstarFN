#ifndef DELAYEDTERMINATIONCONDITION_H
#define DELAYEDTERMINATIONCONDITION_H

#include <ompl/base/PlannerTerminationCondition.h>

namespace ompl {
namespace base {
class DelayedTerminationCondition {
 public:
  DelayedTerminationCondition();

  /**
   * @brief eval
   * @return
   */
  bool eval();

  /**
   * @brief signal is used to inform that
   */
  void signal();

  /**
   * @brief terminate
   */
  void terminate();

  /**
   * @brief operator PlannerTerminationCondition
   */
  operator PlannerTerminationCondition();

private:
  int timesCalled_;
  int percentDelay_;
  int delayedCalls_;
  mutable bool isDelay_;
};
}
}

#endif  // DELAYEDTERMINATIONCONDITION_H
