#include <iostream>

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
    std::uniform_int_distribution<int> uni(-1000,1000); // guaranteed unbiased

    World* myWorld = SkelParser::readWorld(
                SAFESPACE_DATA"/ground_plane/ground.skel");
    Skeleton* staubli
            = SoftSdfParser::readSkeleton(SAFESPACE_DATA"/safespace/model.sdf");

    staubli->disableSelfCollision();

    Skeleton* obstacle[5];

    for (int i = 0; i < 5; ++i) {

        obstacle[i] = SkelParser::readSkeleton(SAFESPACE_DATA"/obstacles/cube.skel");



        auto x = uni(rng);
        auto y = uni(rng);
        auto z = uni(rng);
        Eigen::Isometry3d T;


        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd((x+y)/1000.0*M_PI, Vector3d::UnitX())
          * Eigen::AngleAxisd((x+2*y-z)/1000.0*M_PI,  Vector3d::UnitY())
          * Eigen::AngleAxisd((x-y+3*z)/1000.0*M_PI, Vector3d::UnitZ());

        T = Eigen::Translation3d(x/1000.0, y/1000.0, z/1000.0+1.0);

        T.rotate(m);
        //cout << T << endl;
        obstacle[i]->getJoint("joint 1")->setTransformFromParentBodyNode(T);
        //obst
    }


    myWorld->addSkeleton(staubli);
    for (int i = 0; i < 8; ++i) {
        staubli->getJoint(i)->setActuatorType(Joint::LOCKED);
        cout << staubli->getJoint(i)->isKinematic() << endl;
        if(i>=2){
            staubli->setPosition(i, 0.6 * DART_PI);
        }
    }

    myWorld->setGravity(Vector3d(0.0, 0.0, -9.81));
    myWorld->getSkeleton(1)->computeForwardKinematics();

    dart::collision::FCLMeshCollisionDetector collisionDetector;
    //collisionDetector.addSkeleton(myWorld->getSkeleton(1));

    dart::collision::FCLMeshCollisionNode table(myWorld->getSkeleton(1)->getBodyNode("table"));
    dart::collision::FCLMeshCollisionNode arm(myWorld->getSkeleton(1)->getBodyNode("arm_link"));
    dart::collision::FCLMeshCollisionNode elbow(myWorld->getSkeleton(1)->getBodyNode("elbow_link"));

    cout << "table<->sholder " << collisionDetector.detectCollision(&table,&elbow,false) << endl;
    cout << "table<->arm " << collisionDetector.detectCollision(&table,&arm,false) << endl;

    //collisionDetector.detectCollision(true, false);
    //   collisionDetector.getCollisionNode(myWorld->getSkeleton(1)->getBodyNode("table"));
    //collisionDetector.

    //    obstacle[0]->
    for (int i = 0; i < 5; ++i) {
        myWorld->addSkeleton(obstacle[i]);
    }


    MyWindow window;

    window.setWorld(myWorld);

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Staubli TX90XL");
    glutMainLoop();

    return 0;
}
