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
Manipulator::Manipulator() :
    pathNodes_(0), goalBias_(0), planningTime_(0)
{
    init();
}

void Manipulator::init()
{
    ds::WorldPtr myWorld(du::SkelParser::readWorld(SAFESPACE_DATA "/ground_plane/ground.skel"));

    dd::SkeletonPtr staubli(du::SoftSdfParser::readSkeleton(SAFESPACE_DATA "/safespace/model.sdf"));

    dd::SkeletonPtr complexObstacle(du::SoftSdfParser::readSkeleton(
                                        SAFESPACE_DATA "/obstacles/complex_obstacle.sdf"));

    // staubli->disableSelfCollision();

    std::string name = "box ";

    for (int i = 0; i < NUM_OBSTACLE; ++i) {
        if (i < 1) {
            myObstacle[i] = du::SkelParser::readSkeleton(SAFESPACE_DATA "/obstacles/wall.skel");
        }
        else if (i < 2) {
            myObstacle[i] = du::SkelParser::readSkeleton(SAFESPACE_DATA "/obstacles/human_box.skel");
        }
        else {
            myObstacle[i] = du::SkelParser::readSkeleton(SAFESPACE_DATA "/obstacles/cube.skel");
        }

        Eigen::Isometry3d T;

        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(obstacle::rpy[i][0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(obstacle::rpy[i][1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(obstacle::rpy[i][2], Eigen::Vector3d::UnitZ());

        T = Eigen::Translation3d(obstacle::pos[i][0], obstacle::pos[i][1],
                obstacle::pos[i][2]);

        T.rotate(m);

        myObstacle[i]->getJoint("joint 1")->setTransformFromParentBodyNode(T);
        name[3] = i + '0';

        myObstacle[i]->setName(name);
        //std::cout << "obstacle name: " << name << std::endl;
    }

    myWorld->addSkeleton(staubli);
    // TODO make it smarter
    // complexObstacle->setName("box4");
    // myWorld->addSkeleton(complexObstacle);

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

    std::cout << myWorld->getTimeStep() << std::endl;

    //*/
    /*
Eigen::Isometry3d T;
T = myObstacle[1]->getBodyNode("box")->getTransform();

T.translation()(0) += 0.1*15;

myObstacle[1]->getJoint("joint 1")->setTransformFromParentBodyNode(T);
myObstacle[1]->computeForwardKinematics();
//

/*
     dart::dynamics::Skeleton *humanBox  = mWorld->getSkeleton("box1");
        Eigen::Isometry3d T;
        T = humanBox->getBodyNode("box")->getTransform();

        T.translation()(0) -= 0.0005;
        T.translation()(1) -= 0.00025;

        humanBox->getJoint("joint 1")->setTransformFromParentBodyNode(T);
 */

    /*
VectorXd q = staubli->getPositions() * 0;

q[2] =  0.7*DART_PI;
q[3] =  0.4*DART_PI;
q[4] = -0.4*DART_PI;
q[5] = 0;
q[6] = 0;
q[7] = 0;

staubli->setPositions(q);
staubli->computeForwardKinematics();
*/

    // myWorld->setTimeStep(0.005);

    // std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

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
    ;

    jointSpace->setup();
    //ss_->setPlanner(ob::PlannerPtr(new og::RRTstar(ss_->getSpaceInformation())));

    staubli_ = world_->getSkeleton("TX90XLHB")->clone();

    table_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("table"));
    base_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("base_link"));
    shoulder_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("shoulder_link"));
    arm_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("arm_link")); //
    elbow_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("elbow_link"));
    forearm_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("forearm_link")); //
    wrist_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("wrist_link"));
    toolflange_link_ = new dc::FCLMeshCollisionNode(staubli_->getBodyNode("toolflange_link"));

    for (int i = 0; i < NUM_OBSTACLE; ++i) {
        name[3] = i + '0';
        obstacle_[i] = new dc::FCLMeshCollisionNode(world_->getSkeleton(name)->getBodyNode(0));
    }
}

//==============================================================================
bool Manipulator::isStateValid(const ob::State* state)
{
    /*
    namespace bc = boost::chrono;
    bc::thread_clock::time_point start = bc::thread_clock::now();
*/
    boost::lock_guard<boost::mutex> guard(mutex_);

    double* jointSpace
            = (double*)state->as<ob::RealVectorStateSpace::StateType>()->values;

    std::vector<dc::Contact> contact;
    int num_max_contact = 1;

    for (int i = 2; i < 8; ++i) {
        staubli_->setPosition(i, jointSpace[i - 2]);
        //std::cout << i << "=" <<jointSpace[i-2] << std::endl;
    }

    staubli_->computeForwardKinematics(true, false, false);
    bool collision;
    /*
    collision = world_->checkCollision(true);
    return collision;
*/

    collision = table_->detectCollision(elbow_link_, &contact, num_max_contact);
    if (collision) {
#ifdef DEBUG
        std::cout << "BAD STATE!" << std::endl;
#endif
        return false;
    }

    collision = table_->detectCollision(forearm_link_, &contact, num_max_contact);
    if (collision) {
#ifdef DEBUG
        std::cout << "BAD STATE!" << std::endl;
#endif
        return false;
    }

    for (int i = 0; i < NUM_OBSTACLE; ++i) {
        collision = forearm_link_->detectCollision(obstacle_[i], &contact, num_max_contact);
        if (collision) {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif

            return false;
        }
        collision = arm_link_->detectCollision(obstacle_[i], &contact, num_max_contact);
        if (collision) {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }
        collision = elbow_link_->detectCollision(obstacle_[i], &contact, num_max_contact);
        if (collision) {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }
        collision = toolflange_link_->detectCollision(obstacle_[i], &contact, num_max_contact);
        if (collision) {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }
        collision = wrist_link_->detectCollision(obstacle_[i], &contact, num_max_contact);
        if (collision) {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }
    }

    for (int i = 0; i < NUM_OBSTACLE; ++i) {
        collision = shoulder_link_->detectCollision(obstacle_[i], &contact, num_max_contact);
        if (collision) {
#ifdef DEBUG
            std::cout << "BAD STATE!" << std::endl;
#endif
            return false;
        }

        collision = base_link_->detectCollision(obstacle_[i], &contact, num_max_contact);
        if (collision) {
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
        OMPL_INFORM("joint %d: %f", i + 1, jointSpace[i]);
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
    start[0] = 62.5 / 180.0 * DART_PI;
    start[1] = 49.5 / 180.0 * DART_PI;
    start[2] = 92.8 / 180.0 * DART_PI;
    start[3] = 0.0 / 180.0 * DART_PI;
    start[4] = 0.0 / 180.0 * DART_PI;
    start[5] = 0;
    ob::ScopedState<> goal(ss_->getStateSpace());
    goal[0] = -52.2 / 180.0 * DART_PI;
    goal[1] = 60.8 / 180.0 * DART_PI;
    goal[2] = 63.0 / 180.0 * DART_PI;
    goal[3] = 0;
    goal[4] = 53.3 / 180.0 * DART_PI;
    goal[5] = 0;
    ss_->setStartAndGoalStates(start, goal);
    // generate a few solutions; all will be added to the goal;

    if (ss_->getPlanner()) {
        ss_->getPlanner()->clear();
        //ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(2.0/180.0*M_PI);
        ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(20.0 / 180.0 * M_PI);
        ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(goalBias_);
        ss_->solve(planningTime_);
    }

    const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
    OMPL_INFORM("Found %d solutions", (int)ns);
    return ss_->haveSolutionPath();
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
    start[0] = 62.5 / 180.0 * DART_PI;
    start[1] = 49.5 / 180.0 * DART_PI;
    start[2] = 92.8 / 180.0 * DART_PI;
    start[3] = 0.0 / 180.0 * DART_PI;
    start[4] = 0.0 / 180.0 * DART_PI;
    start[5] = 0;
    ob::ScopedState<> goal(ss_->getStateSpace());
    goal[0] = -52.2 / 180.0 * DART_PI;
    goal[1] = 60.8 / 180.0 * DART_PI;
    goal[2] = 63.0 / 180.0 * DART_PI;
    goal[3] = 0;
    goal[4] = 53.3 / 180.0 * DART_PI;
    goal[5] = 0;

    ss_->setStartAndGoalStates(start, goal);
    ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(2.5 / 180.0 * M_PI);
    ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(goalBias_);
    ss_->getPlanner()->as<og::DRRTstarFN>()->restoreTree(filename);

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
        //OMPL_WARN("Hey intermediate state is invalid");
        return false;
    }

    for (double step = 0.1; step < 1; step += 0.1) {
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
void Manipulator::setPlanningTime(int time)
{
    planningTime_ = time;
}
//==============================================================================
int Manipulator::getPlanningTime()
{
    return planningTime_;
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
void Manipulator::setGoalBias(double bias)
{
    goalBias_ = bias;
}
//==============================================================================
void Manipulator::setPathNodes(int pathNodes)
{
    pathNodes_ = pathNodes;
}
//==============================================================================
og::PathGeometric Manipulator::getResultantMotion()
{
    if (!ss_ || !ss_->haveSolutionPath()) {
        OMPL_WARN("No solution");
        og::PathGeometric p(ss_->getSpaceInformation());
        return p;
    }

    og::PathGeometric& p = ss_->getSolutionPath();
    p.interpolate(pathNodes_);
    return p;
}