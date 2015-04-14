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

#include "mywindow.h"

#define SAFESPACE_DATA "/home/olzhas/devel/staubli_dart/data/"

using namespace std;
using namespace Eigen;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;


int main(int argc, char* argv[])
{
    World* myWorld = SkelParser::readWorld(
                SAFESPACE_DATA"/ground_plane/ground.skel");
    Skeleton* staubli
            = SoftSdfParser::readSkeleton(SAFESPACE_DATA"/safespace/model.sdf");

    myWorld->addSkeleton(staubli);
    for (int i = 0; i < 8; ++i) {
        staubli->getJoint(i)->setActuatorType(Joint::LOCKED);
        cout << staubli->getJoint(i)->isKinematic() << endl;
        if(i>=2){
            staubli->setPosition(i, 0.5 * DART_PI);
        }
    }

    myWorld->setGravity(Vector3d(0.0, 0.0, -9.81));
    myWorld->getSkeleton(1)->computeForwardKinematics();

    dart::collision::FCLCollisionDetector collisionDetector;
    collisionDetector.addSkeleton(myWorld->getSkeleton(1));

    MyWindow window;

    window.setWorld(myWorld);

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Staubli TX90XL");
    glutMainLoop();


    return 0;
}
