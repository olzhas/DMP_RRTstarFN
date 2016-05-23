#include "DynamicMotionPlanningSetup.h"

DynamicMotionPlanningSetup::DynamicMotionPlanningSetup(ompl::base::SpaceInformationPtr &si) :
    ompl::geometric::SimpleSetup(si)
{

}


DynamicMotionPlanningSetup::DynamicMotionPlanningSetup(ompl::base::StateSpacePtr &space) :
    ompl::geometric::SimpleSetup(space)
{

}


void DynamicMotionPlanningSetup::setup()
{
    timestep = 100;
}

void DynamicMotionPlanningSetup::clear()
{

}

void DynamicMotionPlanningSetup::plan()
{

}
