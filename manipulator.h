#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalRegion.h>
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
#include <ompl/util/RandomNumbers.h>

#include <dart/dart.h>

#include <boost/filesystem.hpp>

#include <vector>
#include <iostream>
#include <fstream>
#include <mutex>

#include "mywindow.h"
#include "configuration.h"
#include "ompl/base/spaces/WeightedRealVectorStateSpace.h"
#include "obstaclemanager.h"
#include "ompl/geometric/planners/rrt/DRRTstarFN.h"

#define NUM_OBSTACLE 5
#define SAFESPACE_DATA "/home/olzhas/devel/staubli_dart/data/"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dc = dart::collision;
namespace dd = dart::dynamics;

class Manipulator {
public:
    og::SimpleSetupPtr ss_;
    //    ob::SpaceInformationPtr si_;

    Manipulator();

    virtual ~Manipulator();
    bool plan();
    bool replan();
    void updateObstacles();

    void setMaxNodes(int nodeNum)
    {
#ifdef DEBUG
        std::cout << ss_->getPlanner()->as<og::RRTstarFN>()->getMaxNodes() << std::endl;
#endif
        ss_->getPlanner()->as<og::DRRTstarFN>()->setMaxNodes(nodeNum);
#ifdef DEBUG
        std::cout << ss_->getPlanner()->as<og::RRTstarFN>()->getMaxNodes() << std::endl;
#endif
    }

    void printEdge(std::ostream& os, const ob::StateSpacePtr& space, const ob::PlannerDataVertex& vertex);
    void recordSolution();

    void setStartState(const std::vector<double>& st);
    void setFinalState(const std::vector<double>& st);

    void init(ConfigurationPtr& config);

    void store(const char* filename);
    void load(const char* filename);

    dart::simulation::WorldPtr getWorld() { return world_; }
    void setWorld(dart::simulation::WorldPtr& world) { world_ = world; }

    ConfigurationPtr cfg;

    MyWindowPtr pWindow;
    dd::SkeletonPtr staubli_;
    ObstacleManager obsManager;

private:
    bool isStateValid(const ob::State* state);
    //bool localReplanFromScratch();
    bool localReplan();
    bool newReplan();

    std::string& dumpFileNameGenerate();
    dart::simulation::WorldPtr world_;

    std::mutex mutex_;

    void configurePlanner();

    void spawnStaticObstacles();

    void spawnObstacle(std::string path);

    void setState(ob::ScopedState<>& state, std::vector<double>& set);

    double cost(const ob::State*, const ob::State*);

    // service routines
    //dart::common::Timer timer1;
};

typedef std::shared_ptr<Manipulator> ManipulatorPtr;

#endif // MANIPULATOR_H_
