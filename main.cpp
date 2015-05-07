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

        if ( i < 1) {
            myObstacle[i] = SkelParser::readSkeleton(SAFESPACE_DATA"/obstacles/wall.skel");
        } else if (i < 2) {
            myObstacle[i] = SkelParser::readSkeleton(SAFESPACE_DATA"/obstacles/human_box.skel");
        } else {
            myObstacle[i] = SkelParser::readSkeleton(SAFESPACE_DATA"/obstacles/cube.skel");
        }

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

    for (int i = 0; i < NUM_OBSTACLE; ++i) {
        myWorld->addSkeleton(myObstacle[i]);
    }

    myWorld->setGravity(Vector3d(0.0, 0.0, 0.0));

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    Manipulator env(myWorld);


    env.setPlanningTime(120);

    env.setMaxNodes(40000);

    if (env.plan()) {
        env.recordSolution();
    }

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

    MyWindow window;

    window.setWorld(myWorld);
    og::PathGeometric resultantMotion = env.getResultantMotion();

    window.setMotion(&resultantMotion);
    window.ss_ = env.ss_;
    window.initDrawTree();

    glutInit(&argc, argv);
    window.initWindow(800, 600, "Staubli TX90XL");
    glutMainLoop();

    return 0;
}
