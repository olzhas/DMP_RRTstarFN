#include <iostream>

// DART
#include <dart/dart.h>

#include "mywindow.h"
#include "manipulator.h"
#include "guimisc.h"

int main(int argc, char* argv[]) {

    Manipulator manipulator;

    manipulator.setPlanningTime(60*30);
    manipulator.setGoalBias(0.0);
    std::cout << "Planning time is set to " << manipulator.getPlanningTime() << "sec\n";
    // env.setPlanningTime(2);
    // env.setPlanningTime(600);

    manipulator.setMaxNodes(7500);
    std::string fileName = "mydump";

//#define LOAD_PRECALC_DATA
#ifndef LOAD_PRECALC_DATA
    if (manipulator.plan()) {
        manipulator.recordSolution();
    }
    manipulator.store(fileName.c_str());
#else
    OMPL_INFORM("Loading the tree from file %s", fileName.c_str());
    manipulator.load(fileName.c_str());
#endif


//#define DYNAMIC_PLANNING
#ifdef DYNAMIC_PLANNING

    for(int j = 0; j < 5; j++){
        std::cout << "\nreplanning iteration #" << j << std::endl;
        manipulator.updateObstacles();
        manipulator.replan();
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
