#include "manipulator.h"
#include "manipulatormotionvalidator.h"

#include "config/obstacle_config_blue.h"

#include "solutionpath.h"

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
//==============================================================================
Manipulator::~Manipulator()
{
    dtwarn << "good\n";
}

inline std::string genBoxName(int i)
{
    std::string name = "box ";
    name[3] = i + '0';
    return name;
}
//==============================================================================
void Manipulator::init(ConfigurationPtr &config)
{
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

    ob::WeightedRealVectorStateSpace* jointSpace = new ob::WeightedRealVectorStateSpace();
    //jointSpace->localCost = boost::bind(&Manipulator::cost, this, _1, _2);
    //ob::RealVectorStateSpace* jointSpace = new ob::RealVectorStateSpace();

    // first two are connection to the world and table
    for(size_t i=0; i < staubli->getNumJoints(); ++i){
        double lower = staubli->getJoint(i)->getPositionLowerLimit(0);
        double upper = staubli->getJoint(i)->getPositionUpperLimit(0);
        if(upper - lower >= EPSILON){
            jointSpace->addDimension(lower, upper);
        }
    }
    jointSpace->setup();
    ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(jointSpace)));
    // set state validity checking for this space
    ss_->setStateValidityChecker(boost::bind(&Manipulator::isStateValid, this, _1));

    //ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / jointSpace->getMaximumExtent());

    ss_->getSpaceInformation()
            ->setMotionValidator(
                ob::MotionValidatorPtr(
                    new ManipulatorMotionValidator(ss_->getSpaceInformation())));

    ss_->setPlanner(ob::PlannerPtr(new og::DRRTstarFN(ss_->getSpaceInformation())));

    ss_->getProblemDefinition()
            ->setOptimizationObjective(
                getPathLengthObjective(ss_->getSpaceInformation()));

    staubli_ = world_->getSkeleton("TX90XLHB");

    obsManager.setPlanWorld(myWorld);
    obsManager.loadAll();
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
double Manipulator::cost(const ob::State* st1, const ob::State* st2)
{
    boost::lock_guard<boost::mutex> guard(mutex_);

    double* js1 = (double*)st1->as<ob::RealVectorStateSpace::StateType>()->values;

    for (int i = 2; i < 8; ++i) {
        staubli_->setPosition(i, js1[i - 2]);
        //std::cout << i << "=" <<jointSpace[i-2] << std::endl;
    }
    staubli_->computeForwardKinematics(true, false, false);
    Eigen::Isometry3d transform1 = staubli_->getBodyNode("toolflange_link")->getTransform();
    Eigen::Vector3d t1 = transform1.translation();

    double* js2 = (double*)st2->as<ob::RealVectorStateSpace::StateType>()->values;

    for (int i = 2; i < 8; ++i) {
        staubli_->setPosition(i, js2[i - 2]);
        //std::cout << i << "=" <<jointSpace[i-2] << std::endl;
    }
    staubli_->computeForwardKinematics(true, false, false);
    Eigen::Isometry3d transform2 = staubli_->getBodyNode("toolflange_link")->getTransform();
    Eigen::Vector3d t2 = transform2.translation();

    double c = 0.0;
    for(int i=0; i<3; ++i){
        double diff = t1[i] - t2[i];
        c += diff*diff;
    }
    return sqrt(c);
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

    SolutionPath* sp = new SolutionPath("main");
    try {
        og::PathGeometric& p = ss_->getSolutionPath();
#ifdef INTERP
        p.interpolate(2000);
#endif
        sp->set(p, ss_->getSpaceInformation(), staubli_);
        pWindow->drawables.push_back(&sp->getDrawables());
        pWindow->solutionPaths.push_back(sp);
    }
    catch (ompl::Exception e) {
        delete sp;
        dtwarn << "No solution, man\n";
    }

    return ss_->haveSolutionPath();
}
//==============================================================================
void Manipulator::configurePlanner()
{
    ss_->getPlanner()->as<og::DRRTstarFN>()->setMaxNodes(cfg->maxNumberNodes);
    ss_->getPlanner()->as<og::DRRTstarFN>()->setDelayCC(true); // delay collision detection
    ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(cfg->rangeRad);
    ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(cfg->goalBias);
}
//==============================================================================
bool Manipulator::replan()
{
    return localReplan();
}
//==============================================================================
#ifdef LOCAL_SUB_REPLAN
bool Manipulator::localReplanFromScratch()
{

    //    if (!ss_)
    //        return false;
    //    // generate a few solutions; all will be added to the goal;
    //    si_ = ss_->getSpaceInformation();
    //    og::PathGeometric& p = ss_->getSolutionPath();

    //    bool* collisionMap = new bool[p.getStateCount()];
    //    int startPos=-1, endPos=-1;

    //    std::list<ompl::base::State*> partition;
    //    for(size_t i(0); i < p.getStateCount(); ++i){
    //        double* jointSpace = (double*)p.getState(i)
    //                ->as<ob::RealVectorStateSpace::StateType>()
    //                ->values;

    //        Eigen::Vector6d tempState;
    //        for(size_t j(0); j<6; ++j){
    //            tempState[j]=jointSpace[j];
    //        }

    //        pWindow->solutionStates.push_back(tempState);
    //        collisionMap[i] = !isStateValid(p.getState(i));
    //        if(startPos > 0 && endPos < 0){
    //            //ompl::base::State* copy = si_->allocState();
    //            //si_->copyState(copy, p.getState(i));
    //            partition.push_back(si_->cloneState(p.getState(i)));
    //        }
    //        if(collisionMap[i] == true && collisionMap[i-1] == false){
    //            startPos = i-1;
    //        }

    //        if(collisionMap[i] == false && collisionMap[i-1] == true){
    //            endPos = i+1;
    //        }

    //    }
    //    cfg->pathCollisionMap = collisionMap;
    //    cfg->pathCollisionMapSize = partition.size();

    //    ConfigurationPtr subConfig(new Configuration);
    //    ManipulatorPtr subProblem(new Manipulator());


    //    if (ss_->getPlanner()) {
    //        //ss_->getPlanner()->as<og::DRRTstarFN>()->
    //        int removed;
    //        if(startPos > 0 && endPos > 0) {

    //            ob::ScopedState<> start(ss_->getStateSpace());
    //            start = si_->cloneState(p.getState(startPos));

    //            ob::ScopedState<> goal(ss_->getStateSpace());
    //            goal = si_->cloneState(p.getState(endPos));


    //            /* defining sub problem */

    //            subConfig->readFile();

    //            subProblem->init(subConfig);
    //            subConfig->startState.clear();

    //            double* tempStartState = (double*)p.getState(startPos)->
    //                    as<ob::RealVectorStateSpace::StateType>()->values;

    //            subConfig->startState.assign(tempStartState, tempStartState+6);

    //            subConfig->goalState.clear();
    //            double* tempGoalState = (double*)p.getState(endPos)->
    //                    as<ob::RealVectorStateSpace::StateType>()->values;
    //            subConfig->goalState.assign(tempGoalState, tempGoalState+6);
    //            subConfig->planningTime = 70;
    //            subConfig->rangeDeg = 50;
    //            subConfig->rangeRad =  subConfig->rangeDeg / 180.0 * M_PI;

    //            subProblem->cfg->goalBias = 0.25;
    //            subProblem->spawnDynamicObstacles();
    //            subProblem->plan();

    //            og::PathGeometric& subp = subProblem->ss_->getSolutionPath();
    //            subp.interpolate();

    //            if (subp.getStateCount() > 0) {
    //                pWindow->subSolutionSetup_ = subProblem->ss_;
    //                pWindow->subSolution.clear();
    //                pWindow->subSolution.reserve(subp.getStateCount());
    //                for (int j(0); j < subp.getStateCount(); j++) {
    //                    double* jointSpace = (double*)subp.getState(j)
    //                            ->as<ob::RealVectorStateSpace::StateType>()
    //                            ->values;
    //                    Eigen::Vector6d tempState;
    //                    for(size_t i(0); i<6; ++i){
    //                        tempState[i] = jointSpace[i];
    //                    }
    //                    pWindow->subSolutionStates.push_back(tempState);

    //                    for (int i = 2; i < 8; ++i) {
    //                        staubli_->setPosition(i, jointSpace[i - 2]);
    //                    }
    //                    staubli_->computeForwardKinematics(true, false, false);
    //                    Eigen::Isometry3d transform = staubli_->getBodyNode("toolflange_link")->getTransform();
    //                    pWindow->subSolution.push_back(transform.translation());
    //                }
    //            }
    //        }

    //        ob::ScopedState<> start(ss_->getStateSpace());
    //        for (std::size_t i(0); i < cfg->startState.size(); ++i) {
    //            start[i] = cfg->startState[i];
    //        }

    //        ob::ScopedState<> goal(ss_->getStateSpace());
    //        for (std::size_t i(0); i < cfg->goalState.size(); ++i) {
    //            goal[i] = cfg->goalState[i];
    //        }
    //        ss_->setStartAndGoalStates(start, goal);
    //        pWindow->ss_ = ss_;

    //        //pWindow->initDrawTree();
    //        //delete cfg->pathCollisionMap;
    //        //cfg->pathCollisionMap = NULL;
    //    }

    //    while(true); // FIXME crashes after this function is finished
    //    //ss_->getProblemDefinition()->clearSolutionPaths();
    //    const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
    //    OMPL_INFORM("Found %d solutions", (int)ns);

    //    return ss_->haveSolutionPath();
}
#endif
//==============================================================================
bool Manipulator::localReplan()
{
    ob::SpaceInformationPtr si(ss_->getSpaceInformation());
    ob::State* interimState = si->allocState();

    og::PathGeometric &p = ss_->getSolutionPath();
    ss_->getPlanner()->as<og::DRRTstarFN>()->setSampleRadius(20.0/180.0 * M_PI);
    ss_->getPlanner()->as<og::DRRTstarFN>()->setLocalPlanning(true);
    ss_->getPlanner()->as<og::DRRTstarFN>()->markForRemoval();
    ss_->getPlanner()->as<og::DRRTstarFN>()->removeNodes();
    OMPL_INFORM("removed nodes");
    //ss_->getPlanner()->as<og::DRRTstarFN>()->stepTwo();
    OMPL_INFORM("step two");
    ss_->getProblemDefinition()->clearSolutionPaths();
    ss_->solve(5);
    OMPL_INFORM("done");
    cfg->dynamicReplanning = true;

    SolutionPath* sp = new SolutionPath("sub", "r");
    try {
        og::PathGeometric& p = ss_->getSolutionPath();

        p.interpolate(200);

        sp->set(p, ss_->getSpaceInformation(), staubli_);
        pWindow->drawables.push_back(&sp->getDrawables());
        pWindow->solutionPaths.push_back(sp);
    }
    catch (ompl::Exception e) {
        delete sp;
        dtwarn << "No solution, man\n";
    }

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

    SolutionPath* sp = new SolutionPath("main");
    try {
        og::PathGeometric& p = ss_->getSolutionPath();
#ifdef INTERP
        p.interpolate(2000);
#endif
        sp->set(p, ss_->getSpaceInformation(), staubli_);
        pWindow->solutionPaths.push_back(sp);
        pWindow->drawables.push_back(&sp->getDrawables());
    }
    catch (ompl::Exception e) {
        delete sp;
        dtwarn << "No solution, man\n";
    }
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
std::string& Manipulator::dumpFileNameGenerate()
{
    std::time_t now = std::time(nullptr);
    std::string* out = new std::string;
    char buffer[]="2015-12-06_01-23-40";
    if(buffer == NULL){
        dtwarn << "Could not generate dump file name\n";
        return *out;
    }
    if(strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", localtime(&now))){
        out->assign(buffer);
    }
    return *out;
}
