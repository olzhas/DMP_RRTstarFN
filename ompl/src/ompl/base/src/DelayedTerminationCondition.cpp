#include "ompl/base/DelayedTerminationCondition.h"

namespace ompl {
namespace base {
DelayedTerminationCondition::DelayedTerminationCondition()
    : timesCalled_(0), percentDelay_(0), delayedCalls_(0), isDelay_(false) {}

bool DelayedTerminationCondition::eval() {
  if (isDelay_) {
    delayedCalls_--;
    if (delayedCalls_ == 0) {
      return false;
    }
  } else {
    timesCalled_++;
  }
  return true;
}

void DelayedTerminationCondition::signal(){
    isDelay_ = true;
}

void DelayedTerminationCondition::terminate()
{
;
}

}
}
