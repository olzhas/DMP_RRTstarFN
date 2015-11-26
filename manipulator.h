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

class Manipulator {
public:
    og::SimpleSetupPtr ss_;
    ob::SpaceInformationPtr si_;

    Manipulator();

    virtual ~Manipulator();
    bool plan();
    bool replan();
    void updateObstacles();

    void setMaxNodes(int nodes);

    void printEdge(std::ostream& os, const ob::StateSpacePtr& space, const ob::PlannerDataVertex& vertex);
    void recordSolution();

    og::PathGeometric* getResultantMotion();

    void setStartState(const std::vector<double>& st);
    void setFinalState(const std::vector<double>& st);

    void init(ConfigurationPtr &config);

    void store(const char* filename);
    void load(const char* filename);

    dart::simulation::WorldPtr getWorld();
    void setWorld(dart::simulation::WorldPtr &world);
    ConfigurationPtr cfg;

    enum ObstacleType { WALL,
                        HUMAN_BBOX,
                        CUBE };
    ObstacleType obstacleStatic[5] = { WALL, HUMAN_BBOX, CUBE, CUBE, CUBE }; // FIXME number of obstacles is fixed

private:
    bool isStateValid(const ob::State* state);

    std::string dumpFileNameGenerate();
    dart::simulation::WorldPtr world_;

    dd::SkeletonPtr staubli_;

    boost::mutex mutex_;

    dd::SkeletonPtr myObstacle[NUM_OBSTACLE];

    void configurePlanner();

    void spawnStaticObstacles();
    void spawnDynamicObstacles();
    void spawnObstacle(std::string path);

};

class ManipulatorMotionValidator : public ob::MotionValidator {
public:
    ManipulatorMotionValidator(ob::SpaceInformation* si)
        : MotionValidator(si)
    {
        defaultSettings();
    }
    ManipulatorMotionValidator(const ob::SpaceInformationPtr& si)
        : MotionValidator(si)
    {
        defaultSettings();
    }
    virtual ~ManipulatorMotionValidator()
    {
    }
    virtual bool checkMotion(const ob::State* s1, const ob::State* s2) const;
    virtual bool checkMotion(const ob::State* s1, const ob::State* s2, std::pair<ob::State*, double>& lastValid) const;

private:
    ob::StateSpacePtr stateSpace_;
    void defaultSettings();
};

typedef std::shared_ptr<Manipulator> ManipulatorPtr;

#endif // MANIPULATOR_H_
