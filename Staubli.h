#ifndef STAUBLI_H
#define STAUBLI_H

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstarFN.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
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
#include <boost/chrono/thread_clock.hpp>
#include <boost/asio/deadline_timer.hpp>

#include <vector>
#include <iostream>
#include <fstream>


#define NUM_OBSTACLE 5

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dc = dart::collision;
namespace dd = dart::dynamics;

class Manipulator 
{
public:

    og::SimpleSetupPtr ss_;

    Manipulator(dart::simulation::World* world);

    virtual ~Manipulator();
    bool plan();

    void printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex);
    void recordSolution();

    og::PathGeometric getResultantMotion();
    void setPlanningTime(int time);
    void setMaxNodes(int nodeNum);

private:

    bool isStateValid(const ob::State *state) const;
    dart::simulation::World* world_;

    void setWorld(dart::simulation::World* world);

    dd::Skeleton *staubli_;
    dc::FCLMeshCollisionNode *table_;
    dc::FCLMeshCollisionNode *base_link_;
    dc::FCLMeshCollisionNode *shoulder_link_;
    dc::FCLMeshCollisionNode *arm_link_;
    dc::FCLMeshCollisionNode *elbow_link_;
    dc::FCLMeshCollisionNode *forearm_link_;
    dc::FCLMeshCollisionNode *wrist_link_;
    dc::FCLMeshCollisionNode *toolflange_link_;

    dc::FCLMeshCollisionNode *obstacle_[NUM_OBSTACLE];

    int planningTime_;

    //boost::asio::deadline_
};


#endif
