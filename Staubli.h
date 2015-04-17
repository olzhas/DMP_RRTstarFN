#ifndef STAUBLI_H
#define STAUBLI_H

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstarFN.h>
#include <ompl/config.h>

#include <dart/config.h>
#include <dart/collision/collision.h>
#include <dart/common/common.h>
#include <dart/constraint/constraint.h>
#include <dart/dynamics/dynamics.h>
#include <dart/integration/integration.h>
#include <dart/lcpsolver/lcpsolver.h>
#include <dart/math/math.h>
#include <dart/renderer/renderer.h>
#include <dart/simulation/simulation.h>
#include <dart/gui/gui.h>
#include <dart/optimizer/optimizer.h>
#include <dart/planning/planning.h>
#include <dart/utils/utils.h>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <vector>
#include <iostream>
#include <fstream>

//#include "dhparameters.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Manipulator 
{
public:

	og::SimpleSetupPtr ss_;


    Manipulator(dart::simulation::World* world);

	virtual ~Manipulator();
	bool plan();

	void printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex);
	void recordSolution();

private:

	bool isStateValid(const ob::State *state) const;
    dart::simulation::World* world_;

    void setWorld(dart::simulation::World* world);

};

#endif
