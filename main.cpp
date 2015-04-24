#include <iostream>
#include <fstream>


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

#include <random>

#include "mywindow.h"
#include "Staubli.h"
#include "guimisc.h"

//#include "config/obstacle_config_red.h"
//#include "config/obstacle_config_green.h"
#include "config/obstacle_config_blue.h"


#define SAFESPACE_DATA "/home/olzhas/devel/staubli_dart/data/"

using namespace std;
using namespace Eigen;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;



int main(int argc, char* argv[])
{

    std::random_device rd;     // only used once to initialise (seed) engine
    std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
    std::uniform_real_distribution<double> uni(-1.0, 1.0); // guaranteed unbiased

    World* myWorld = SkelParser::readWorld(
                SAFESPACE_DATA"/ground_plane/ground.skel");
    Skeleton* staubli
            = SoftSdfParser::readSkeleton(SAFESPACE_DATA"/safespace/model.sdf");

    staubli->disableSelfCollision();

    Skeleton* myObstacle[NUM_OBSTACLE];
    std::string name = "box ";

    for (int i = 0; i < NUM_OBSTACLE; ++i) {

        if ( i < 2){
            myObstacle[i] = SkelParser::readSkeleton(SAFESPACE_DATA"/obstacles/wall.skel");
        } else if (i < 4) {
            myObstacle[i] = SkelParser::readSkeleton(SAFESPACE_DATA"/obstacles/human_box.skel");
        } else {
            myObstacle[i] = SkelParser::readSkeleton(SAFESPACE_DATA"/obstacles/cube.skel");
        }

        /*
        auto x = uni(rng);
        auto y = uni(rng);
        auto z = uni(rng);
        */
        Eigen::Isometry3d T;

        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(obstacle::rpy[i][0], Vector3d::UnitX())
                * Eigen::AngleAxisd(obstacle::rpy[i][1],  Vector3d::UnitY())
                * Eigen::AngleAxisd(obstacle::rpy[i][2], Vector3d::UnitZ());

        T = Eigen::Translation3d(obstacle::pos[i][0],
                obstacle::pos[i][1],
                obstacle::pos[i][2]);

        T.rotate(m);

        myObstacle[i]->getJoint("joint 1")->setTransformFromParentBodyNode(T);
        name[3] = i+'0';
        myObstacle[i]->setName(name);

    }

    myWorld->addSkeleton(staubli);
    for (int i = 0; i < 8; ++i) {
        staubli->getJoint(i)->setActuatorType(Joint::LOCKED);
#ifdef DEBUG
        cout << staubli->getJoint(i)->isKinematic() << endl;
#endif

    }

    //Shape *obs;
    //obs = myObstacle[4]->getBodyNode(0)->getVisualizationShape(0);
    //obs->setColor(Vector3d(0.9,0.1,0.01));
    //obstacle[1]->getBod
    for (int i = 0; i < NUM_OBSTACLE; ++i) {
        myWorld->addSkeleton(myObstacle[i]);
    }

    myWorld->setGravity(Vector3d(0.0, 0.0, 0.0));

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    Manipulator env(myWorld);

    env.setPlanningTime(1200);

    if (env.plan()) {
        env.recordSolution();
    }
/*
    ifstream fin("vertices.dat");
    double visuState[6];
    while(!fin.eof()){
        for (int i = 0; i < 6; ++i) {
            fin >> visuState[i];
            staubli->setPosition(i+2, visuState[i]);
        }
        staubli->computeForwardKinematics();

        std::cout << staubli->getJoint("toolflange_link")->getLocalTransform()<< std::endl;
    }
    */
    //std::cout << "BV " << staubli->getBodyNode("table")->getNumCollisionShapes() << std::endl;

    VectorXd q = staubli->getPositions() * 0;
    q[2] = -0.01*DART_PI;
    q[3] = 0.4*DART_PI;
    q[4] = -0.6*DART_PI;
    q[5] = 0;
    q[6] = 0;
    q[7] = 0;

    staubli->setPositions(q);
    staubli->computeForwardKinematics();
    MyWindow window;

    window.setWorld(myWorld);
    og::PathGeometric resultantMotion = env.getResultantMotion();
    window.setMotion(&resultantMotion);
    //window.jointStates =



    glutInit(&argc, argv);
    window.initWindow(800, 600, "Staubli TX90XL");
    glutMainLoop();

    return 0;
}
