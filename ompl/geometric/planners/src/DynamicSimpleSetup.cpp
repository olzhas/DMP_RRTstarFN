#include "DynamicSimpleSetup.h"

DynamicSimpleSetup::DynamicSimpleSetup(ompl::base::SpaceInformationPtr &si) :
    ompl::geometric::SimpleSetup(si)
{

}


DynamicSimpleSetup::DynamicSimpleSetup(ompl::base::StateSpacePtr &space) :
    ompl::geometric::SimpleSetup(space)
{

}


void DynamicSimpleSetup::setup()
{
    timestep = 100;
}

void DynamicSimpleSetup::clear()
{

}

void DynamicSimpleSetup::plan()
{

}
