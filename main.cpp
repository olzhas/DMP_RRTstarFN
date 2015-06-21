#include <iostream>

// DART
#include <dart/dart.h>

#include "mywindow.h"
#include "manipulator.h"
#include "guimisc.h"

int main(int argc, char* argv[]) {

    Manipulator manipulator;

    manipulator.setPlanningTime(1200);
    std::cout << "Planning time is set to " << manipulator.getPlanningTime() << "sec\n";
    // env.setPlanningTime(2);
    // env.setPlanningTime(600);

    manipulator.setMaxNodes(30000);
    std::string fileName = "mydump";

//#define LOAD_PRECALC_DATA
#ifndef LOAD_PRECALC_DATA
    if (manipulator.plan()) {
        manipulator.recordSolution();
    }
    manipulator.store(fileName.c_str());
#else
    manipulator.load(fileName.c_str());
#endif


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

    MyWindow window;

    window.setWorld(manipulator.getWorld());
    og::PathGeometric resultantMotion = manipulator.getResultantMotion();

    window.setMotion(&resultantMotion);
    window.ss_ = manipulator.ss_;
    window.initDrawTree();

    glutInit(&argc, argv);
    window.initWindow(800, 600, "Staubli TX90XL");
    glutMainLoop();

    return 0;
}
