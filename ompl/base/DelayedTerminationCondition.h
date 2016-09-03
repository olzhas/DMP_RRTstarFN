#ifndef DELAYEDTERMINATIONCONDITION_H
#define DELAYEDTERMINATIONCONDITION_H

#include <ompl/base/PlannerTerminationCondition.h>

namespace ompl {
namespace base {
class DelayedTerminationCondition {
 public:
  DelayedTerminationCondition();

  /** \brief */
  bool eval();

  /** \brief */
  void signal();

  /** \brief */
  void terminate();

  /** \brief */
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
