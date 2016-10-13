#ifndef DYNAMICPLANNER_H
#define DYNAMICPLANNER_H

#include <ompl/base/Planner.h>

namespace ompl {
namespace base {
class DynamicPlanner : Planner {
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

  ///
  /// \brief getName
  /// \return
  ///
  std::string getName() const { return name_; }

  ///
  /// \brief setName
  /// \param name
  ///
  void setName(const std::string &name) { name_ = name; }

 private:
  std::string name_;
  SpaceInformationPtr si_;
};

typedef std::shared_ptr<DynamicPlanner> DynamicPlannerPtr;
}
}

#endif  // DYNAMICPLANNER_H
