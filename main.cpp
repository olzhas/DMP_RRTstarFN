#include <memory>
#include <iostream>
#include <boost/thread.hpp>

// DART
#include <dart/dart.h>

#include "widget.h"
#include "manipulator.h"
#include "guimisc.h"

int main(int argc, char* argv[]) {

    boost::thread guiThread;

    ManipulatorPtr manipulator(new Manipulator);
    std::string fileName = "mydump";
    manipulator->setPathNodes(3000);

#define LOAD_PRECALC_DATA
#ifndef LOAD_PRECALC_DATA
    manipulator->setPlanningTime(60*30);
    manipulator->setGoalBias(0.0);
    manipulator->setMaxNodes(5000);
    std::cout << "Planning time is set to " << manipulator.getPlanningTime() << "sec\n";

    if (manipulator->plan()) {
        manipulator->recordSolution();
    }
    manipulator->store(fileName.c_str());
#else
    OMPL_INFORM("Loading the tree from file %s", fileName.c_str());
    manipulator->load(fileName.c_str());
#endif


//#define DYNAMIC_PLANNING
#ifdef DYNAMIC_PLANNING

    for(int j = 0; j < 5; j++){
        std::cout << "\nreplanning iteration #" << j << std::endl;
        manipulator->updateObstacles();
        manipulator->replan();
    }
#endif

    Widget widget;
    widget.setManipulator(manipulator);
    widget.init();
    widget.exec(&argc, argv);

    return 0;
}
