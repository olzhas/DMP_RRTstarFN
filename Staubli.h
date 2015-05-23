#ifndef STAUBLI_H_
#define STAUBLI_H_

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/DRRTstarFN.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Exception.h>
#include <ompl/config.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

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
#include <boost/chrono/thread_clock.hpp>
#include <boost/thread.hpp>

#include <vector>
#include <iostream>
#include <fstream>
#include <mutex>


#define NUM_OBSTACLE 5

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dc = dart::collision;
namespace dd = dart::dynamics;

class Manipulator 
{
    public:

    og::SimpleSetupPtr ss_;
    ob::SpaceInformationPtr si_;

    Manipulator(dart::simulation::World* world);

    virtual ~Manipulator();
    bool plan();
    bool replan();

    void printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex);
    void recordSolution();

    og::PathGeometric getResultantMotion();
    void setPlanningTime(int time);
    void setMaxNodes(int nodeNum);

private:

    bool isStateValid(const ob::State *state);
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

    boost::mutex mutex_;
};

class ManipulatorMotionValidator : public ob::MotionValidator
{
public:
    ManipulatorMotionValidator(ob::SpaceInformation *si) : MotionValidator(si)
    {
        defaultSettings();
    }
    ManipulatorMotionValidator(const ob::SpaceInformationPtr &si) : MotionValidator(si)
    {
        defaultSettings();
    }
    virtual ~ManipulatorMotionValidator()
    {
    }
    virtual bool checkMotion(const ob::State *s1, const ob::State *s2) const;
    virtual bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State*, double> &lastValid) const;
private:
    ob::StateSpacePtr stateSpace_;
    void defaultSettings();
};

#endif // STAUBLI_H_
