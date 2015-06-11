#include <iostream>
#include <fstream>

// DART
#include <dart/dart.h>

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

int main(int argc, char* argv[]) {
  World* myWorld =
      SkelParser::readWorld(SAFESPACE_DATA "/ground_plane/ground.skel");

  // myWorld->getConstraintSolver()->setCollisionDetector(
  //    new dart::collision::BulletCollisionDetector());

  Skeleton* staubli =
      SoftSdfParser::readSkeleton(SAFESPACE_DATA "/safespace/model.sdf");

  Skeleton* complexObstacle = SoftSdfParser::readSkeleton(
      SAFESPACE_DATA "/obstacles/complex_obstacle.sdf");

  // staubli->disableSelfCollision();

  Skeleton* myObstacle[NUM_OBSTACLE];
  std::string name = "box ";

  for (int i = 0; i < NUM_OBSTACLE; ++i) {
    if (i < 1) {
      myObstacle[i] =
          SkelParser::readSkeleton(SAFESPACE_DATA "/obstacles/wall.skel");
    } else if (i < 2) {
      myObstacle[i] =
          SkelParser::readSkeleton(SAFESPACE_DATA "/obstacles/human_box.skel");
    } else {
      myObstacle[i] =
          SkelParser::readSkeleton(SAFESPACE_DATA "/obstacles/cube.skel");
    }

    Eigen::Isometry3d T;

    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(obstacle::rpy[i][0], Vector3d::UnitX()) *
        Eigen::AngleAxisd(obstacle::rpy[i][1], Vector3d::UnitY()) *
        Eigen::AngleAxisd(obstacle::rpy[i][2], Vector3d::UnitZ());

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
    staubli->getJoint(i)->setActuatorType(Joint::LOCKED);
#ifdef DEBUG
    cout << staubli->getJoint(i)->isKinematic() << endl;
#endif
  }

  for (int i = 0; i < NUM_OBSTACLE; ++i) {
    myWorld->addSkeleton(myObstacle[i]);
  }

  myWorld->setGravity(Vector3d(0.0, 0.0, 0.0));
  // myWorld->setTimeStep(0.005);

  // std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  Manipulator env(myWorld);

  env.setPlanningTime(10);
  // env.setPlanningTime(2);
  // env.setPlanningTime(600);

  env.setMaxNodes(1000);
  std::string fileName = "mydump";

#define LOAD_PRECALC_DATA
#ifndef LOAD_PRECALC_DATA
  if (env.plan()) {
    env.recordSolution();
  }
  env.store(fileName.c_str());
#else
  env.load(fileName.c_str());
#endif
  std::cout << myWorld->getTimeStep() << std::endl;

/*

    ompl::base::PlannerDataStorage plannerDataStorage;
    ompl::base::PlannerDataPtr plannerData;
    plannerDataStorage.store(*plannerData, fileName.c_str());
*/

//#define DYNAMIC_PLANNING
#ifdef DYNAMIC_PLANNING

  // double avgSpeed = 0.05;// calculated from the average speed of walking, 5
  // kph
  double avgSpeed = 0.1;
  for (int j = 0; j < 1; j++) {
    Eigen::Isometry3d T;
    T = myObstacle[1]->getBodyNode("box")->getTransform();

    T.translation()(0) -= avgSpeed;

    myObstacle[1]->getJoint("joint 1")->setTransformFromParentBodyNode(T);
    myObstacle[1]->computeForwardKinematics(true, false, false);
    std::cout << "\nreplanning iteration #" << j << std::endl;
    env.replan();
  }
#endif

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
