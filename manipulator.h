#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

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
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>

#include <dart/dart.h>
#include <dart/common/Uri.h>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/chrono/thread_clock.hpp>
#include <boost/thread.hpp>

#include <vector>
#include <iostream>
#include <fstream>
#include <mutex>

#include "configuration.h"

#define NUM_OBSTACLE 5
#define SAFESPACE_DATA "/home/olzhas/devel/staubli_dart/data/"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dc = dart::collision;
namespace dd = dart::dynamics;

class Manipulator 
{
public:

    og::SimpleSetupPtr ss_;
    ob::SpaceInformationPtr si_;

    Manipulator();

    virtual ~Manipulator();
    bool plan();
    bool replan();
    void updateObstacles();

    void printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex);
    void recordSolution();

    og::PathGeometric* getResultantMotion();
    void setPlanningTime(int time);
    int getPlanningTime();
    void setGoalBias(double bias);
    void setMaxNodes(int nodeNum);
    void setPathNodes(int pathNodes);
    void setRange(double range);

    void setStartState(const std::vector<double> &st);
    void setFinalState(const std::vector<double> &st);

    void init(const Configuration &config);

    void store(const char *filename);
    void load(const char *filename);

    dart::simulation::WorldPtr getWorld();
    void setWorld(dart::simulation::WorldPtr world);

private:

    bool isStateValid(const ob::State *state);
    dart::simulation::WorldPtr world_;

    dd::SkeletonPtr staubli_;

    dd::SkeletonPtr staubliInit_;
    dd::SkeletonPtr staubliFinal_;

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
    double goalBias_;
    int pathNodes_;
    double range_;
    std::vector<double> startState_;
    std::vector<double> goalState_;

    Configuration cfg;

    boost::mutex mutex_;

    dd::SkeletonPtr myObstacle[NUM_OBSTACLE];
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

typedef std::shared_ptr<Manipulator> ManipulatorPtr;

#endif // MANIPULATOR_H_
