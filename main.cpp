#include <memory>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// DART
#include <dart/dart.h>

#include "widget.h"
#include "manipulator.h"
#include "guimisc.h"

int main(int argc, char* argv[]) {

    ManipulatorPtr manipulator(new Manipulator);
    std::string fileName = "mydump";
    manipulator->setPathNodes(3000);

    Widget widget;
    widget.setManipulator(manipulator);
    widget.init();

    boost::thread guiThread(boost::bind(&Widget::exec, widget, &argc, argv));
    //widget.exec(&argc, argv);

    //#define LOAD_PRECALC_DATA
#ifndef LOAD_PRECALC_DATA
    manipulator->setPlanningTime(60*5);
    manipulator->setGoalBias(0.0);
    manipulator->setMaxNodes(15000);
    std::cout << "Planning time is set to " << manipulator->getPlanningTime() << "sec\n";

    if (manipulator->plan()) {
        manipulator->recordSolution();
        manipulator->store(fileName.c_str());
    }
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

    //boost::thread guiThread(boost::bind(&Widget::exec, widget, &argc, argv));
    guiThread.join();

    return 0;
}
