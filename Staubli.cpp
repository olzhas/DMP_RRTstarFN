#include "Staubli.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dc = dart::collision;
namespace dd = dart::dynamics;

/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

//==============================================================================
Manipulator::Manipulator(dart::simulation::World* world)
{
    setWorld(world);
    ob::RealVectorStateSpace *jointSpace = new ob::RealVectorStateSpace();

    jointSpace->addDimension(-M_PI, M_PI);
    jointSpace->addDimension(-130.0/180.0*M_PI, 147.5/180.0*M_PI);
    jointSpace->addDimension(-145.0/180.0*M_PI, 145.0/180.0*M_PI);
    jointSpace->addDimension(-270.0/180.0*M_PI, 270.0/180.0*M_PI);
    jointSpace->addDimension(-115.0/180.0*M_PI, 140.0/180.0*M_PI);
    jointSpace->addDimension(-270.0/180.0*M_PI, 270.0/180.0*M_PI);

    ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(jointSpace)));
    // set state validity checking for this space
    ss_->setStateValidityChecker(boost::bind(&Manipulator::isStateValid, this, _1));

    ss_->getSpaceInformation()
            ->setStateValidityCheckingResolution(1.0 / jointSpace->getMaximumExtent());

    ss_->getSpaceInformation()
            ->setMotionValidator(
                ob::MotionValidatorPtr(
                    new ManipulatorMotionValidator(ss_->getSpaceInformation())
                    ));

    ss_->setPlanner(ob::PlannerPtr(new og::DRRTstarFN(ss_->getSpaceInformation())));

    ss_->getProblemDefinition()
            ->setOptimizationObjective(
                getPathLengthObjective(ss_->getSpaceInformation()));;

    jointSpace->setup();
    //ss_->setPlanner(ob::PlannerPtr(new og::RRTstar(ss_->getSpaceInformation())));

    staubli_ = world_->getSkeleton("TX90XLHB");

    table_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("table"));
    base_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("base_link"));
    shoulder_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("shoulder_link"));
    arm_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("arm_link"));//
    elbow_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("elbow_link"));
    forearm_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("forearm_link"));//
    wrist_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("wrist_link"));
    toolflange_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("toolflange_link"));

    std::string name = "box0";
    for (int i = 0; i < NUM_OBSTACLE; ++i) {
        name[3] = i + '0';
        obstacle_[i] = new dc::FCLMeshCollisionNode(world_->getSkeleton(name)->getBodyNode(0));
    }
}

//==============================================================================
bool Manipulator::isStateValid(const ob::State *state) const
{
    /*
    namespace bc = boost::chrono;
    bc::thread_clock::time_point start = bc::thread_clock::now();
*/

    int num_max_contact = 5;
    double *jointSpace
            = (double*)state->as<ob::RealVectorStateSpace::StateType>()->values;

    for (int i = 2; i < 8; ++i)
        staubli_->setPosition(i, jointSpace[i-2]);

    staubli_->computeForwardKinematics();

    bool collision = table_->detectCollision(elbow_link_, NULL, num_max_contact);
    if (collision){
#ifdef DEBUG
        std::cout << "BAD STATE!" << std::endl;
#endif
        return false;
    }

    collision = table_->detectCollision(forearm_link_, NULL, num_max_contact);
    if (collision){
#ifdef DEBUG
        std::cout << "BAD STATE!" << std::endl;
#endif
        return false;
    }

    for (int i = 0; i < NUM_OBSTACLE; ++i) {
        collision = forearm_link_->detectCollision(obstacle_[i], NULL, num_max_contact);
        if (collision)
        {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }
        collision = arm_link_->detectCollision(obstacle_[i], NULL, num_max_contact);
        if (collision)
        {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }
        collision = elbow_link_->detectCollision(obstacle_[i], NULL, num_max_contact);
        if (collision)
        {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }
        collision = shoulder_link_->detectCollision(obstacle_[i], NULL, num_max_contact);
        if (collision)
        {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }
        collision = wrist_link_->detectCollision(obstacle_[i], NULL, num_max_contact);
        if (collision)
        {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }
        collision = toolflange_link_->detectCollision(obstacle_[i], NULL, num_max_contact);
        if (collision)
        {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }
        collision = base_link_->detectCollision(obstacle_[i], NULL, num_max_contact);
        if (collision)
        {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }
    }
    return true;


    //bool collision_test = table_->detectCollision(&elbow_link_new, NULL, 1);
    /*
    switch(staubli->getBodyNode("shoulder_link")->getCollisionShape(0)->getShapeType()) {
    case dd::Shape::BOX:
        std::cout << "Shape::BOX" << std::endl;
        break;
    case dd::Shape::ELLIPSOID:
        std::cout << "Shape::ELLIPSOID" << std::endl;
        break;
    case dd::Shape::CYLINDER:
        std::cout << "Shape::CYLINDER" << std::endl;
        break;
    case dd::Shape::PLANE:
        std::cout << "Shape::CYLINDER" << std::endl;
        break;
    case dd::Shape::MESH:
        std::cout << "Shape::MESH" << std::endl;
        break;
    case dd::Shape::SOFT_MESH:
        std::cout << "Shape::SOFT_MESH" << std::endl;
        break;
    default:
        std::cout << "nothing to watch here" << std::endl;
    }


    bc::thread_clock::time_point stop = bc::thread_clock::now();
    std::cout << "duration: "
              << bc::duration_cast<bc::milliseconds>(stop - start).count()
              << " ms\n";
*/

#ifdef DEBUG
    for (int i = 0; i < 6; ++i) {
        OMPL_INFORM("joint %d: %f", i+1, jointSpace[i]);
    }
#endif

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
    start[0] = 62.5/180.0*DART_PI;
    start[1] = 49.5/180.0*DART_PI;
    start[2] = 92.8/180.0*DART_PI;
    start[3] = 0.0/180.0*DART_PI;
    start[4] = 0.0/180.0*DART_PI;
    start[5] = 0;
    ob::ScopedState<> goal(ss_->getStateSpace());
    goal[0] = -52.2/180.0*DART_PI;
    goal[1] = 60.8/180.0*DART_PI;
    goal[2] = 63.0/180.0*DART_PI;
    goal[3] = 0;
    goal[4] = 53.3/180.0*DART_PI;
    goal[5] = 0;
    ss_->setStartAndGoalStates(start, goal);
    // generate a few solutions; all will be added to the goal;


    if (ss_->getPlanner()){
        ss_->getPlanner()->clear();
        ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(1/180.0*M_PI);
        ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(0.0);
        //ss_->getPlanner()->as<og::RRTstarFN>()->rng_.setLocalSeed(32);
    }
    ss_->solve(planningTime_);

    const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
    OMPL_INFORM("Found %d solutions", (int)ns);
    return ss_->haveSolutionPath();
}

//==============================================================================
void Manipulator::printEdge(std::ostream &os, const ob::StateSpacePtr &space,
                            const ob::PlannerDataVertex &vertex)
{
    std::vector<double> reals;
    if(vertex!=ob::PlannerData::NO_VERTEX)
    {
        space->copyToReals(reals, vertex.getState());
        for(size_t j(0); j<reals.size(); ++j)  os<<" "<<reals[j];
    }
}

//==============================================================================
void Manipulator::recordSolution()
{
    if (!ss_ || !ss_->haveSolutionPath()){
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
    for(unsigned int i(0); i<pdat.numVertices(); ++i)
    {
        printEdge(ofs_v, ss_->getStateSpace(), pdat.getVertex(i));
        ofs_v<<std::endl;
    }

    // Print the edges to file
    std::ofstream ofs_e("edges.dat");
    std::vector<unsigned int> edge_list;
    for(unsigned int i(0); i<pdat.numVertices(); ++i)
    {
        unsigned int n_edge= pdat.getEdges(i,edge_list);
        for(unsigned int i2(0); i2<n_edge; ++i2)
        {
            printEdge(ofs_e, ss_->getStateSpace(), pdat.getVertex(i));
            ofs_e<<std::endl;
            printEdge(ofs_e, ss_->getStateSpace(), pdat.getVertex(edge_list[i2]));
            ofs_e<<std::endl;
            ofs_e<<std::endl<<std::endl;
        }
    }
}

//==============================================================================
void Manipulator::setWorld(dart::simulation::World *world)
{
    world_ = world;
}

//==============================================================================
og::PathGeometric Manipulator::getResultantMotion()
{
    if (!ss_ || !ss_->haveSolutionPath())
    {
        OMPL_ERROR("No solution");
    }

    og::PathGeometric &p = ss_->getSolutionPath();
    //p.interpolate(50);
    return p;
}

//==============================================================================
void Manipulator::setPlanningTime(int time)
{
    planningTime_ = time;
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
bool Manipulator::replan()
{
    if (!ss_)
        return false;
    // generate a few solutions; all will be added to the goal;

    for (int i = 0 ; i < 1 ; ++i)
    {
        if (ss_->getPlanner()) {
            OMPL_WARN("range %f", 1.0/180.0*M_PI);
            ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(1/180.0*M_PI);
            ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(0);
            //ss_->getPlanner()->as<og::RRTstarFN>()->rng_.setLocalSeed(32);
        }
        ss_->solve(1);
    }
    const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
    OMPL_INFORM("Found %d solutions", (int)ns);
    return ss_->haveSolutionPath();
}


//==============================================================================
// TODO implement motion validator
bool ManipulatorMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
{
    ob::State *s3;
    if (si_->isValid(s1) == false || si_->isValid(s2) == false ){
        OMPL_WARN("Hey intermediate state is invalid");
        return false;
    }

    stateSpace_->as<ob::RealVectorStateSpace>()->interpolate(s1, s2, 0.25, s3);
    if (si_->isValid(s3) == false ){
        OMPL_WARN("Hey intermediate state1 is invalid");
        return false;
    }

    stateSpace_->as<ob::RealVectorStateSpace>()->interpolate(s1, s2, 0.50, s3);
    if (si_->isValid(s3) == false ){
        OMPL_WARN("Hey intermediate state2 is invalid");
        return false;
    }

    stateSpace_->as<ob::RealVectorStateSpace>()->interpolate(s1, s2, 0.75, s3);
    if (si_->isValid(s3) == false ){
        OMPL_WARN("Hey intermediate state3 is invalid");
        return false;
    }

    return true;
}

//==============================================================================
// TODO implement motion validator
bool ManipulatorMotionValidator::checkMotion(const ob::State *s1,
                                             const ob::State *s2,
                                             std::pair<ob::State*, double> &lastValid) const
{
    /*
    ob::State *s3;
    if (ss_->getSpaceInformation()->isValid(s1) == false
            || ss_->getSpaceInformation()->isValid(s2) == false )
        return false;


    ss_->getStateSpace()->as<RealVectorStateSpace>()->interpolate(s1, s2, 0.25, s3);
    if (ss_->getSpaceInformation()->isValid(s3) == false )
        return false;

    ss_->getStateSpace()->as<RealVectorStateSpace>()->interpolate(s1, s2, 0.50, s3);
    if (ss_->getSpaceInformation()->isValid(s3) == false )
        return false;

    ss_->getStateSpace()->as<RealVectorStateSpace>()->interpolate(s1, s2, 0.75, s3);
    if (ss_->getSpaceInformation()->isValid(s3) == false )
        return false;

    return true;
    */
    return false;
}

void ManipulatorMotionValidator::defaultSettings()
{
    stateSpace_ = si_->getStateSpace();
    if (!stateSpace_)
        throw ompl::Exception("No state space for motion validator");
}
