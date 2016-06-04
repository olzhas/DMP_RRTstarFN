#ifndef DYNAMICSIMPLESETUP_H
#define DYNAMICSIMPLESETUP_H

#include <chrono>

#include <ompl/geometric/SimpleSetup.h>

namespace ompl {
namespace geometric {

class DynamicSimpleSetup : public ompl::geometric::SimpleSetup {
 public:
  explicit DynamicSimpleSetup(ompl::base::SpaceInformationPtr &si);

  explicit DynamicSimpleSetup(ompl::base::StateSpacePtr &space);

  /** \brief */
  void loadPrecomputed();

  /** \brief drive the robot */
  bool drive();

  /** \brief user supplied prepeartion routine*/
  void (*prepareUser)();

  void (*reactUser)();

  void (*clearUser)();

 private:
  /** \brief time step between regular obstacle collision routine in
   * milliseconds */

  std::chrono::milliseconds timestep;  // dt

  /** \brief call back for obstacle collision routine */
  bool (*collisionChecker)();

  /** \brief reaction routine */
  void react();

  /** \brief reset to default value of the algorithm */
  void setup();

  void clear();

  /** \brief work on problem */
  void plan();

  /** \brief preparation step for the dynamic motion planning */
  void prepare();

  void move();

  bool validSolution();

  void stop();

  bool loadPrecomputedData_=false;
};

typedef std::shared_ptr<DynamicSimpleSetup> DynamicSimpleSetupPtr;
}
}

#endif  // DYNAMICMOTIONPLANNINGSETUP_H
