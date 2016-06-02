#ifndef DYNAMICSIMPLESETUP_H
#define DYNAMICSIMPLESETUP_H

#include <ompl/geometric/SimpleSetup.h>


class DynamicSimpleSetup : public ompl::geometric::SimpleSetup
{
public:
    explicit
    DynamicSimpleSetup(ompl::base::SpaceInformationPtr &si);

    explicit
    DynamicSimpleSetup(ompl::base::StateSpacePtr &space);

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

    /** \brief preparation step for the dynamic motion planning */
    void prepare();

    /** \brief drive the robot */
    void drive();

};


#endif // DYNAMICMOTIONPLANNINGSETUP_H
