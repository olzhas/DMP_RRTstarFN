#include "manipulator.h"
#include "manipulatormotionvalidator.h"

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
//==============================================================================
void Manipulator::spawnObstacle(std::string path)
{
    ;
}
//==============================================================================
// FIXME reimplement it with spawnObstacle() method
void Manipulator::spawnStaticObstacles()
{
    dd::SkeletonPtr myObstacle[NUM_OBSTACLE];
    for (int i = 0; i < cfg->numObstacle; ++i) {

        std::string obstaclePath(SAFESPACE_DATA);
        switch (obstacleStatic[i]) {
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

    for (int i = 0; i < cfg->numObstacle; ++i) {
        world_->addSkeleton(myObstacle[i]);
    }
}
//==============================================================================
void Manipulator::init(ConfigurationPtr &config)
{
    ompl::RNG::setSeed(10);
    cfg = config;
    ds::WorldPtr myWorld(du::SkelParser::readWorld(
                             dart::common::Uri::createFromString(
                                 SAFESPACE_DATA "/ground_plane/ground.skel")));
    dd::SkeletonPtr staubli(du::SdfParser::readSkeleton(SAFESPACE_DATA "/safespace/model.sdf"));

    staubli->enableSelfCollision();
    myWorld->addSkeleton(staubli);

    for (int i = 0; i < 8; ++i) {
        staubli->getJoint(i)->setActuatorType(dd::Joint::LOCKED);

#ifdef DEBUG
        cout << staubli->getJoint(i)->isKinematic() << endl;
#endif
    }

    myWorld->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));

    setWorld(myWorld);
    spawnStaticObstacles();

    ob::WeightedRealVectorStateSpace* jointSpace = new ob::WeightedRealVectorStateSpace();

    // TODO make this configurable through config files
    jointSpace->addDimension(-M_PI, M_PI);
    jointSpace->addDimension(-130.0 / 180.0 * M_PI, 147.5 / 180.0 * M_PI);
    jointSpace->addDimension(-145.0 / 180.0 * M_PI, 145.0 / 180.0 * M_PI);
    jointSpace->addDimension(-270.0 / 180.0 * M_PI, 270.0 / 180.0 * M_PI);
    jointSpace->addDimension(-115.0 / 180.0 * M_PI, 140.0 / 180.0 * M_PI);
    jointSpace->addDimension(-270.0 / 180.0 * M_PI, 270.0 / 180.0 * M_PI);

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
    for (std::size_t i(0); i < cfg->startState.size(); ++i) {
        start[i] = cfg->startState[i];
    }

    ob::ScopedState<> goal(ss_->getStateSpace());
    for (std::size_t i(0); i < cfg->goalState.size(); ++i) {
        goal[i] = cfg->goalState[i];
    }
    ss_->setStartAndGoalStates(start, goal);
    // generate a few solutions; all will be added to the goal;

    if (ss_->getPlanner()) {
        ss_->getPlanner()->clear();
        configurePlanner();
        ss_->solve(cfg->planningTime);
    }

    const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
    OMPL_INFORM("Found %d solutions", (int)ns);
    return ss_->haveSolutionPath();
}
//==============================================================================
void Manipulator::configurePlanner()
{
    ss_->getPlanner()->as<og::DRRTstarFN>()->setMaxNodes(cfg->maxNumberNodes);
    ss_->getPlanner()->as<og::DRRTstarFN>()->setDelayCC(false); // FIXME configuation value
    ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(cfg->rangeRad);
    ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(cfg->goalBias);
}
//==============================================================================
// TODO
std::string dumpFileNameGenerate()
{

}
//==============================================================================
bool Manipulator::replan()
{
    if (!ss_)
        return false;
    // generate a few solutions; all will be added to the goal;
    si_ = ss_->getSpaceInformation();
    og::PathGeometric& p = ss_->getSolutionPath();

    bool* collisionMap = new bool[p.getStateCount()];
    int startPos=-1, endPos=-1;

    std::list<ompl::base::State*> partition;
    for(size_t i(0); i < p.getStateCount(); ++i){
        collisionMap[i] = !isStateValid(p.getState(i));
        if(startPos > 0 && endPos < 0){
            //ompl::base::State* copy = si_->allocState();
            //si_->copyState(copy, p.getState(i));
            partition.push_back(si_->cloneState(p.getState(i)));
        }
        if(collisionMap[i] == true && collisionMap[i-1] == false){
            startPos = i-1;
        }

        if(collisionMap[i] == false && collisionMap[i-1] == true){
            endPos = i+1;
        }

    }
    cfg->pathCollisionMap = collisionMap;

    if (ss_->getPlanner()) {
        //ss_->getPlanner()->as<og::DRRTstarFN>()->
        int removed;
        if(startPos > 0 && endPos > 0) {

            ob::ScopedState<> start(ss_->getStateSpace());
            start = si_->cloneState(p.getState(startPos));

            ob::ScopedState<> goal(ss_->getStateSpace());
            goal = si_->cloneState(p.getState(endPos));

            ob::State* interimState = si_->allocState();
            //ob::State* startState = &state;
            //ob::State* goa
            ss_->getStateSpace()->as<ob::RealVectorStateSpace>()->
                    interpolate(p.getState(startPos), p.getState(endPos),
                                0.5, interimState);
            ss_->getPlanner()->as<og::DRRTstarFN>()->setInterimState(interimState);
            double radius = ss_->getStateSpace()->
                    as<ob::RealVectorStateSpace>()->
                    distance(p.getState(startPos), p.getState(endPos));
            ss_->getPlanner()->as<og::DRRTstarFN>()->setSampleRadius(0.3);
            ss_->getPlanner()->as<og::DRRTstarFN>()->setLocalPlanning(true);

            ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(0.8);

            ss_->getProblemDefinition()->clearSolutionPaths();
            ss_->setStartAndGoalStates(start, goal);
            configurePlanner();

            removed =
                    ss_->getPlanner()->as<og::DRRTstarFN>()->removeNodes(partition);

        }
        ss_->solve(30);
        cfg->dynamicReplanning = true;


        ob::ScopedState<> start(ss_->getStateSpace());
        for (std::size_t i(0); i < cfg->startState.size(); ++i) {
            start[i] = cfg->startState[i];
        }

        ob::ScopedState<> goal(ss_->getStateSpace());
        for (std::size_t i(0); i < cfg->goalState.size(); ++i) {
            goal[i] = cfg->goalState[i];
        }
        ss_->setStartAndGoalStates(start, goal);
        pWindow->ss_ = ss_;

        pWindow->initDrawTree();
        delete cfg->pathCollisionMap;
        cfg->pathCollisionMap = NULL;
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
    // FIXME introduce update for dynamic obstacles
    //    double avgSpeed = 0.05;
    //    Eigen::Isometry3d T;
    //    T = myObstacle[1]->getBodyNode("box")->getTransform();

    //    T.translation()(0) -= avgSpeed;

    //    myObstacle[1]->getJoint("joint 1")->setTransformFromParentBodyNode(T);
    //    myObstacle[1]->computeForwardKinematics(true, false, false);
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
// TODO
inline void Manipulator::setState(ob::ScopedState<> &state, std::vector<double> &set)
{
    for(size_t i(0); i<set.size(); ++i){
        state[i] = set[i];
    }
}
//==============================================================================
void Manipulator::load(const char* filename)
{
    if (ss_->getPlanner()) {
        ss_->getPlanner()->clear();
    }

    ss_->setup();

    ob::ScopedState<> start(ss_->getStateSpace());
    setState(start, cfg->startState);

    ob::ScopedState<> goal(ss_->getStateSpace());
    setState(goal, cfg->goalState);

    ss_->setStartAndGoalStates(start, goal);
    configurePlanner();
    ss_->getPlanner()->as<og::DRRTstarFN>()->restoreTree(cfg->loadDataFile.c_str());

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
void Manipulator::spawnDynamicObstacles()
{
    double rpy[][3] = {{ 4.16858072, 3.14151269, 2.59488083},
                       { 5.79539508, 4.12217189, 5.59803713},
                       { 0, 0, 1.7},
                       { 0, 0, 0.7},
                       { 0, 0, 0}};

    double pos[][3] = {{  0.950,  -0.423, 1.844},
                       {  10.192,  0.701, 0.940},
                       {  10.916, -0.517, 1.230},
                       {  10.768, -0.282, 1.623},
                       {  10.200, -0.700, 1.450}};

    std::string obstaclePath(SAFESPACE_DATA "/obstacles/cube.skel");

    dd::SkeletonPtr dynamicObstacle = du::SkelParser::readSkeleton(obstaclePath);

    Eigen::Isometry3d T;
    Eigen::Matrix3d m;

    m  = Eigen::AngleAxisd(rpy[0][0], Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(rpy[0][1], Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(rpy[0][2], Eigen::Vector3d::UnitZ());

    T = Eigen::Translation3d(pos[0][0], pos[0][1], pos[0][2]);

    T.rotate(m);

    dynamicObstacle->getJoint("joint 1")->setTransformFromParentBodyNode(T);
    dynamicObstacle->setName(genBoxName(2)); //FIXME hardcoded value
    dynamicObstacle->getBodyNode(0)->getVisualizationShape(0)->setAlpha(0.9);

    pWindow->getWorld()->addSkeleton(dynamicObstacle);
    world_->addSkeleton(dynamicObstacle);
}
