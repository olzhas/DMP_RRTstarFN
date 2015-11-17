#include "planningproblem.h"

PlanningProblem::PlanningProblem()
{
    loadData = false;
    configure();
}

void PlanningProblem::configure()
{
    manipulator = ManipulatorPtr(new Manipulator);
    manipulator->setPathNodes(3000);
    manipulator->setPlanningTime(60*5);
    manipulator->setGoalBias(0.5);
    manipulator->setMaxNodes(25000);
}

void PlanningProblem::readFile()
{

}

void PlanningProblem::setConfigFileName(std::string &filename)
{
    fileName = filename;
}

int PlanningProblem::exec(int argc, char* argv[])
{
    // just to check if time ticks correctly
    system("date");
    std::string fileName = "mydump";

    Widget widget;
    widget.setManipulator(manipulator);
    widget.init();

    boost::thread guiThread(boost::bind(&Widget::exec, widget, &argc, argv));
    boost::thread planThread(boost::bind(&PlanningProblem::plan, this, &argc, argv));

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
    return EXIT_SUCCESS;
}

void PlanningProblem::plan(int* argcp, char** argv)
{
    if(!loadData){
        std::cout << "Planning time is set to " << manipulator->getPlanningTime() << "sec\n";
        if (manipulator->plan()) {
            manipulator->recordSolution();
            manipulator->store(fileName.c_str());
        }
    } else {
        OMPL_INFORM("Loading the tree from file %s", fileName.c_str());
        manipulator->load(fileName.c_str());
    }
}
