#include "Staubli.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


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
    jointSpace->setup();
    ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / jointSpace->getMaximumExtent());
    ss_->setPlanner(ob::PlannerPtr(new og::RRTstarFN(ss_->getSpaceInformation())));
            //ss_->setPlanner(ob::PlannerPtr(new og::RRTstar(ss_->getSpaceInformation())));

}

bool Manipulator::isStateValid(const ob::State *state) const
{
    double *jointSpace = (double*)state->as<ob::RealVectorStateSpace::StateType>()->values;

    //world-
    dart::dynamics::Skeleton* staubli =
                world_->getSkeleton("TX90XLHB");

    //assert(staubli == NULL);

    for (int i = 2; i < 8; ++i) {
        staubli->setPosition(i, jointSpace[i-2]);
    }

    staubli->computeForwardKinematics();

    dart::collision::FCLMeshCollisionDetector collisionDetector;
    dart::collision::FCLMeshCollisionNode table(staubli->getBodyNode("table"));
    dart::collision::FCLMeshCollisionNode shoulder(staubli->getBodyNode("shoulder_link"));

    return !(collisionDetector.detectCollision(&table, &shoulder, false));

#ifdef DEBUG
    for (int i = 0; i < 6; ++i) {
        OMPL_INFORM("joint %d: %f", i+1, jointSpace[i]);
    }
#endif

}

Manipulator::~Manipulator()
{
    // @TODO
}

bool Manipulator::plan()
{

    if (!ss_)
        return false;
    ob::ScopedState<> start(ss_->getStateSpace());
    start[0] = 0;
    start[1] = 0;
    start[2] = 0;
    start[3] = 0;
    start[4] = 0;
    start[5] = 0;
    ob::ScopedState<> goal(ss_->getStateSpace());
    goal[0] = 0.3;
    goal[1] = 0.5;
    goal[2] = 0.5;
    goal[3] = 0.5;
    goal[4] = 1;
    goal[5] = 1;
    ss_->setStartAndGoalStates(start, goal);
            // generate a few solutions; all will be added to the goal;

    for (int i = 0 ; i < 1 ; ++i)
    {
        if (ss_->getPlanner()){
            ss_->getPlanner()->clear();
            ss_->getPlanner()->as<og::RRTstarFN>()->setRange(1.0/180.0*M_PI);
                    //ss_->getPlanner()->as<og::RRTstarFN>()->rng_.setLocalSeed(32);
        }
        ss_->solve(10);
    }
    const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
    OMPL_INFORM("Found %d solutions", (int)ns);
    return ss_->haveSolutionPath();
}

void Manipulator::printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex)
{
    std::vector<double> reals;
    if(vertex!=ob::PlannerData::NO_VERTEX)
    {
        space->copyToReals(reals, vertex.getState());
        for(size_t j(0); j<reals.size(); ++j)  os<<" "<<reals[j];
    }
}

void Manipulator::recordSolution()
{
    if (!ss_ || !ss_->haveSolutionPath())
        return;

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

void Manipulator::setWorld(dart::simulation::World *world)
{
    world_ = world;
}

