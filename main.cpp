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

/// \brief Function headers
enum TypeOfDOF {
  DOF_X, DOF_Y, DOF_Z, DOF_ROLL, DOF_PITCH, DOF_YAW
};

/// \brief Add a DOF to a given joint
dart::dynamics::Joint* create1DOFJoint(double val, double min, double max,
                                       int type) {
  // Create the transformation based on the type
  dart::dynamics::Joint* newJoint = NULL;
  if (type == DOF_X)
    newJoint = new dart::dynamics::PrismaticJoint(
                 Eigen::Vector3d(1.0, 0.0, 0.0));
  else if (type == DOF_Y)
    newJoint = new dart::dynamics::PrismaticJoint(
                 Eigen::Vector3d(0.0, 1.0, 0.0));
  else if (type == DOF_Z)
    newJoint = new dart::dynamics::PrismaticJoint(
                 Eigen::Vector3d(0.0, 0.0, 1.0));
  else if (type == DOF_YAW)
    newJoint = new dart::dynamics::RevoluteJoint(
                 Eigen::Vector3d(0.0, 0.0, 1.0));
  else if (type == DOF_PITCH)
    newJoint = new dart::dynamics::RevoluteJoint(
                 Eigen::Vector3d(0.0, 1.0, 0.0));
  else if (type == DOF_ROLL)
    newJoint = new dart::dynamics::RevoluteJoint(
                 Eigen::Vector3d(1.0, 0.0, 0.0));
  // Add the transformation to the joint, set the min/max values and set it to
  // the skeleton
  newJoint->setPosition(0, val);
  newJoint->setPositionLowerLimit(0, min);
  newJoint->setPositionUpperLimit(0, max);

  return newJoint;
}

int main(int argc, char* argv[])
{
    /*
    DartLoader urdfLoader;

    Skeleton* ground = urdfLoader.parseSkeleton(
          DART_DATA_PATH"/ground_plane/ground.urdf");
*/
    World* myWorld = new World;
    Skeleton* staubli
            = SoftSdfParser::readSkeleton(SAFESPACE_DATA"/safespace/model.sdf");
    Skeleton* ground
            = SoftSdfParser::readSkeleton(SAFESPACE_DATA"/ground_plane/model-1_4.sdf");

    myWorld->addSkeleton(ground);
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

    MyWindow window;

    window.setWorld(myWorld);


    glutInit(&argc, argv);
    window.initWindow(640, 480, "Staubli TX90XL");
    glutMainLoop();


    return 0;
}
/*
#include <iostream>

#include <dart/dart.h>

#include "apps/atlasSimbicon/MyWindow.h>
#include "apps/atlasSimbicon/Controller.h>



int main(int argc, char* argv[])
{
  // Create empty soft world
  World* myWorld = new World;

  // Load ground and Atlas robot and add them to the world
  DartLoader urdfLoader;
  Skeleton* ground = urdfLoader.parseSkeleton(
        DART_DATA_PATH"sdf/atlas/ground.urdf");
//  Skeleton* atlas = SoftSdfParser::readSkeleton(
//        DART_DATA_PATH"sdf/atlas/atlas_v3_no_head.sdf");
  Skeleton* atlas
      = SoftSdfParser::readSkeleton(
          DART_DATA_PATH"sdf/atlas/atlas_v3_no_head_soft_feet.sdf");
  myWorld->addSkeleton(atlas);
  myWorld->addSkeleton(ground);

  // Set initial configuration for Atlas robot
  VectorXd q = atlas->getPositions();
  q[0] = -0.5 * DART_PI;
  atlas->setPositions(q);
  atlas->computeForwardKinematics(true, true, false);

  // Set gravity of the world
  myWorld->setGravity(Vector3d(0.0, -9.81, 0.0));

  // Create a window and link it to the world
  MyWindow window(new Controller(atlas, myWorld->getConstraintSolver()));
  window.setWorld(myWorld);

  // Print manual
  cout << "space bar: simulation on/off" << endl;
  cout << "'p': playback/stop" << endl;
  cout << "'[' and ']': play one frame backward and forward" << endl;
  cout << "'v': visualization on/off" << endl;
  cout << endl;
  cout << "'h': harness pelvis on/off" << endl;
  cout << "'j': harness left foot on/off" << endl;
  cout << "'k': harness right foot on/off" << endl;
  cout << "'r': reset robot" << endl;
  cout << "'n': transite to the next state manually" << endl;
  cout << endl;
  cout << "'1': standing controller" << endl;
  cout << "'2': walking controller" << endl;

  // Run glut loop
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Atlas Robot");
  glutMainLoop();

  return 0;
}
*/
