#ifndef DYNAMICMOTIONPLANNINGSETUP_H
#define DYNAMICMOTIONPLANNINGSETUP_H


#include <ompl/geometric/SimpleSetup.h>


class DynamicMotionPlanningSetup : public ompl::geometric::SimpleSetup
{
public:
    explicit
    DynamicMotionPlanningSetup(ompl::base::SpaceInformationPtr &si);

    explicit
    DynamicMotionPlanningSetup(ompl::base::StateSpacePtr &space);

private:

    /** \brief time step between regular obstacle collision routine in milliseconds */
    int timestep; // dt

    /** \brief call back for obstacle collision routine */
    bool (*collisionChecker)();

    /** \brief reaction routine */
    void react();

    /** \brief reset to default value of the algorithm */
    void setup();

    void clear();

    /** \brief work on problem */
    void plan();

    /** \brief */
    void prepare();
};


#endif // DYNAMICMOTIONPLANNINGSETUP_H
