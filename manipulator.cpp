#include "manipulator.h"

//#include "config/obstacle_config_red.h"
//#include "config/obstacle_config_green.h"
#include "config/obstacle_config_blue.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dc = dart::collision;
namespace dd = dart::dynamics;
namespace ds = dart::simulation;
namespace du = dart::utils;

/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

//==============================================================================
Manipulator::Manipulator()
{
    ;
}

inline std::string genBoxName(int i)
{
    std::string name = "box ";
    name[3] = i + '0';
    return name;
}

void Manipulator::init(const Configuration& config)
{
    cfg = config;
    ds::WorldPtr myWorld(du::SkelParser::readWorld(
                             dart::common::Uri::createFromString(
                                 SAFESPACE_DATA "/ground_plane/ground.skel")));

    dd::SkeletonPtr staubli(du::SdfParser::readSkeleton(SAFESPACE_DATA "/safespace/model.sdf"));

    /*
    dd::SkeletonPtr complexObstacle(du::SoftSdfParser::readSkeleton(
        SAFESPACE_DATA "/obstacles/complex_obstacle.sdf"));
    */

    enum ObstacleType { WALL,
                        HUMAN_BBOX,
                        CUBE };

    ObstacleType obstType[] = { WALL, HUMAN_BBOX, CUBE, CUBE, CUBE };

    for (int i = 0; i < NUM_OBSTACLE; ++i) {

        std::string obstaclePath(SAFESPACE_DATA);
        switch (obstType[i]) {
        case WALL:
            obstaclePath += "/obstacles/wall.skel";
            break;
        case HUMAN_BBOX:
            obstaclePath += "/obstacles/human_box.skel";
            break;
        case CUBE:
            obstaclePath += "/obstacles/cube.skel";
            break;
        default:
            std::cerr << "Incorrenct obstacle type\n";
        }

        myObstacle[i] = du::SkelParser::readSkeleton(obstaclePath);

        Eigen::Isometry3d T;
        Eigen::Matrix3d m;

        m = Eigen::AngleAxisd(obstacle::rpy[i][0],
                Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(obstacle::rpy[i][1],
                Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(obstacle::rpy[i][2],
                Eigen::Vector3d::UnitZ());

        T = Eigen::Translation3d(obstacle::pos[i][0], obstacle::pos[i][1],
                obstacle::pos[i][2]);

        T.rotate(m);

        myObstacle[i]->getJoint("joint 1")->setTransformFromParentBodyNode(T);
        myObstacle[i]->setName(genBoxName(i));
    }

    myWorld->addSkeleton(staubli);

#ifdef COMPLEX_OBSTACLE
    complexObstacle->setName("box4");
    myWorld->addSkeleton(complexObstacle);
#endif

    for (int i = 0; i < 8; ++i) {
        staubli->getJoint(i)->setActuatorType(dd::Joint::LOCKED);

#ifdef DEBUG
        cout << staubli->getJoint(i)->isKinematic() << endl;
#endif
    }

    for (int i = 0; i < NUM_OBSTACLE; ++i) {
        myWorld->addSkeleton(myObstacle[i]);
    }

    myWorld->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));

    setWorld(myWorld);

    ob::RealVectorStateSpace* jointSpace = new ob::RealVectorStateSpace();

    // TODO make this configurable through config files
    jointSpace->addDimension(-M_PI, M_PI);
    jointSpace->addDimension(-130.0 / 180.0 * M_PI, 147.5 / 180.0 * M_PI);
    jointSpace->addDimension(-145.0 / 180.0 * M_PI, 145.0 / 180.0 * M_PI);
    jointSpace->addDimension(-270.0 / 180.0 * M_PI, 270.0 / 180.0 * M_PI);
    jointSpace->addDimension(-115.0 / 180.0 * M_PI, 140.0 / 180.0 * M_PI);
    jointSpace->addDimension(-270.0 / 180.0 * M_PI, 270.0 / 180.0 * M_PI);
    //jointSpace->setMaximumExtent(10);

    ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(jointSpace)));
    // set state validity checking for this space
    ss_->setStateValidityChecker(boost::bind(&Manipulator::isStateValid, this, _1));

    //    ss_->getSpaceInformation()
    //            ->setStateValidityCheckingResolution(1.0 / jointSpace->getMaximumExtent());

    ss_->getSpaceInformation()
            ->setMotionValidator(
                ob::MotionValidatorPtr(
                    new ManipulatorMotionValidator(ss_->getSpaceInformation())));

    //ss_->setPlanner(ob::PlannerPtr(new og::RRTstar(ss_->getSpaceInformation())));
    ss_->setPlanner(ob::PlannerPtr(new og::DRRTstarFN(ss_->getSpaceInformation())));

    ss_->getProblemDefinition()
            ->setOptimizationObjective(
                getPathLengthObjective(ss_->getSpaceInformation()));

    jointSpace->setup();
    //ss_->setPlanner(ob::PlannerPtr(new og::RRTstar(ss_->getSpaceInformation())));

    staubli_ = world_->getSkeleton("TX90XLHB");
    /*

    for(size_t i(0); i < 8; ++i){
        rbtCollisionNode[i] = new dc::FCLMeshCollisionNode(staubli_->getBodyNode(rbtCollisionNodeName[i]));
    }

    for (int i = 0; i < NUM_OBSTACLE; ++i) {
        obstacle_[i] = new dc::FCLMeshCollisionNode(world_->getSkeleton(genBoxName(i))->getBodyNode(0));
    }
*/
}

//==============================================================================
bool Manipulator::isStateValid(const ob::State* state)
{
    boost::lock_guard<boost::mutex> guard(mutex_);

    double* jointSpace
            = (double*)state->as<ob::RealVectorStateSpace::StateType>()->values;

    for (int i = 2; i < 8; ++i) {
        staubli_->setPosition(i, jointSpace[i - 2]);
        //std::cout << i << "=" <<jointSpace[i-2] << std::endl;
    }

    staubli_->computeForwardKinematics(true, false, false);
    if(world_->checkCollision(false))
        return false;

    return true;
}

//==============================================================================
Manipulator::~Manipulator()
{
    // TODO
}

//==============================================================================
bool Manipulator::plan()
{
    if (!ss_)
        return false;

    ob::ScopedState<> start(ss_->getStateSpace());
    for (std::size_t i(0); i < cfg.startState.size(); ++i) {
        start[i] = cfg.startState[i];
    }

    ob::ScopedState<> goal(ss_->getStateSpace());
    for (std::size_t i(0); i < cfg.goalState.size(); ++i) {
        goal[i] = cfg.goalState[i];
    }
    ss_->setStartAndGoalStates(start, goal);
    // generate a few solutions; all will be added to the goal;

    if (ss_->getPlanner()) {
        ss_->getPlanner()->clear();
        ss_->getPlanner()->as<og::DRRTstarFN>()->setDelayCC(false); // FIXME configuation value
        ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(cfg.rangeRad);
        ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(cfg.goalBias);
        ss_->solve(cfg.planningTime);
    }

    const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
    OMPL_INFORM("Found %d solutions", (int)ns);
    return ss_->haveSolutionPath();
}
//==============================================================================
std::string dumpFileNameGenerate()
{

}
//==============================================================================
bool Manipulator::replan()
{
    if (!ss_)
        return false;
    // generate a few solutions; all will be added to the goal;
    ss_->getProblemDefinition()->clearSolutionPaths();

    if (ss_->getPlanner()) {
        ss_->solve(0.1);
    }

    //ss_->getProblemDefinition()->clearSolutionPaths();
    const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
    OMPL_INFORM("Found %d solutions", (int)ns);

    return ss_->haveSolutionPath();
}
//==============================================================================
void Manipulator::updateObstacles()
{
    // double avgSpeed = 0.05;// calculated from the average speed of walking, 5
    // kph

    double avgSpeed = 0.05;
    Eigen::Isometry3d T;
    T = myObstacle[1]->getBodyNode("box")->getTransform();

    T.translation()(0) -= avgSpeed;

    myObstacle[1]->getJoint("joint 1")->setTransformFromParentBodyNode(T);
    myObstacle[1]->computeForwardKinematics(true, false, false);
}

//==============================================================================
void Manipulator::store(const char* filename)
{
    // Get the planner data to visualize the vertices and the edges
    ob::PlannerData pdat(ss_->getSpaceInformation());
    ss_->getPlannerData(pdat);

    ob::PlannerDataStorage pdstorage;

    pdstorage.store(pdat, filename);
}
//==============================================================================
void Manipulator::load(const char* filename)
{
    if (ss_->getPlanner()) {
        ss_->getPlanner()->clear();
    }

    ss_->setup();

    ob::ScopedState<> start(ss_->getStateSpace());
    for (std::size_t i(0); i < cfg.startState.size(); ++i) {
        start[i] = cfg.startState[i];
    }

    ob::ScopedState<> goal(ss_->getStateSpace());
    for (std::size_t i(0); i < cfg.goalState.size(); ++i) {
        goal[i] = cfg.goalState[i];
    }

    ss_->setStartAndGoalStates(start, goal);
    ss_->getPlanner()->as<og::DRRTstarFN>()->setDelayCC(false);
    ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(cfg.rangeRad);
    ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(cfg.goalBias);
    ss_->getPlanner()->as<og::DRRTstarFN>()->restoreTree(cfg.loadDataFile.c_str());

    /*
    ob::PlannerData pdat(ss_->getSpaceInformation());
    ob::PlannerDataStorage pdstorage;
    pdstorage.load(filename, pdat);

    std::vector<unsigned int> edgeList;
    unsigned int test = 0;
    for (int i = 0; i < pdat.numVertices(); ++i) {
        if (pdat.isStartVertex(i)) {
            std::cout << "start vertex " << i << std::endl;
        }

        pdat.getEdges(i, edgeList);
        for(int j = 0; j < edgeList.size(); ++j) {
            std::cout << edgeList[j] << " ";
        }
        if(edgeList.size() > 0)
            std::cout << "\n";
        test += edgeList.size();
    }
    std::cout << test << " = " << pdat.numVertices() << std::endl;
*/

    //graph = pdat.toBoostGraph();

    /*
    std::vector<unsigned int> edgeList;
    for (int i = 0; i < pdat.numVertices(); ++i) {
        if (  pdat.getVertex(i) != ob::PlannerData::NO_VERTEX){
            pdat.getIncomingEdges(i,edgeList);
            std::cout << edgeList.length() << std::endl;
           for (int j = 0; j < edgeList.length(); ++j) {

            }

       }
        else
            std::cout << "we've got a right node\n";
    }
*/
    //ss_->getPlanner()->as<og::DRRTstarFN>()
}

//==============================================================================
bool ManipulatorMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2) const
{
    ob::State* s3;
    if (si_->isValid(s1) == false || si_->isValid(s2) == false) {
        //OMPL_WARN("Hey, the initial or final state is invalid");
        return false;
    }
#define INTERP_STEP 0.05
    for (double step = INTERP_STEP; step < 1.0; step += INTERP_STEP) {
        stateSpace_->as<ob::RealVectorStateSpace>()->interpolate(s1, s2, step, s3);
        if (si_->isValid(s3) == false) {
            //OMPL_WARN("Hey intermediate state is invalid");
            return false;
        }
    }

    return true;
}

//==============================================================================
// TODO implement motion validator
bool ManipulatorMotionValidator::checkMotion(const ob::State* s1,
                                             const ob::State* s2,
                                             std::pair<ob::State*, double>& lastValid) const
{
    OMPL_ERROR("call of the method");
    return false;
}

void ManipulatorMotionValidator::defaultSettings()
{
    stateSpace_ = si_->getStateSpace();
    if (!stateSpace_)
        throw ompl::Exception("No state space for motion validator");
}

//==============================================================================
void Manipulator::printEdge(std::ostream& os, const ob::StateSpacePtr& space,
                            const ob::PlannerDataVertex& vertex)
{
    std::vector<double> reals;
    if (vertex != ob::PlannerData::NO_VERTEX) {
        space->copyToReals(reals, vertex.getState());
        for (size_t j(0); j < reals.size(); ++j)
            os << " " << reals[j];
    }
}

//==============================================================================
void Manipulator::recordSolution()
{
    if (!ss_ || !ss_->haveSolutionPath()) {
        OMPL_ERROR("No solution!");
        return;
    }

    // Print the solution path to a file
    std::ofstream ofs("path.dat");
    ss_->getSolutionPath().printAsMatrix(ofs);

    // Get the planner data to visualize the vertices and the edges
    ob::PlannerData pdat(ss_->getSpaceInformation());
    ss_->getPlannerData(pdat);

    // Print the vertices to file
    std::ofstream ofs_v("vertices.dat");
    for (unsigned int i(0); i < pdat.numVertices(); ++i) {
        printEdge(ofs_v, ss_->getStateSpace(), pdat.getVertex(i));
        ofs_v << std::endl;
    }

    // Print the edges to file
    std::ofstream ofs_e("edges.dat");
    std::vector<unsigned int> edge_list;
    for (unsigned int i(0); i < pdat.numVertices(); ++i) {
        unsigned int n_edge = pdat.getEdges(i, edge_list);
        for (unsigned int i2(0); i2 < n_edge; ++i2) {
            printEdge(ofs_e, ss_->getStateSpace(), pdat.getVertex(i));
            ofs_e << std::endl;
            printEdge(ofs_e, ss_->getStateSpace(), pdat.getVertex(edge_list[i2]));
            ofs_e << std::endl;
            ofs_e << std::endl
                  << std::endl;
        }
    }
}

//==============================================================================
void Manipulator::setWorld(dart::simulation::WorldPtr world)
{
    world_ = world;
}

//==============================================================================
dart::simulation::WorldPtr Manipulator::getWorld()
{
    return world_;
}
//==============================================================================
void Manipulator::setMaxNodes(int nodeNum)
{
#ifdef DEBUG
    std::cout << ss_->getPlanner()->as<og::RRTstarFN>()->getMaxNodes() << std::endl;
#endif

    ss_->getPlanner()->as<og::DRRTstarFN>()->setMaxNodes(nodeNum);

#ifdef DEBUG
    std::cout << ss_->getPlanner()->as<og::RRTstarFN>()->getMaxNodes() << std::endl;
#endif
}
//==============================================================================
og::PathGeometric* Manipulator::getResultantMotion()
{
    if (!ss_ || !ss_->haveSolutionPath()) {
        OMPL_WARN("No solution");
        return NULL;
    }

    og::PathGeometric& p = ss_->getSolutionPath();
    p.interpolate(cfg.pathNodes);
    return &p;
}
