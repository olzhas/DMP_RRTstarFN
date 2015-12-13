#include "planningproblem.h"

//==============================================================================
PlanningProblem::PlanningProblem():
    cfg(new Configuration),
    manipulator(new Manipulator)
{
    cfg->readFile();

    ompl::RNG::setSeed(cfg->randomSeed);

    manipulator->init(cfg);
}
//==============================================================================
int PlanningProblem::solve(int argc, char* argv[])
{
    //boost::thread dataUpdater(boost::bind());
    boost::thread planThread(boost::bind(&PlanningProblem::plan, this, &argc, argv));

    frontend.setManipulator(manipulator);
    frontend.init();

    boost::thread guiThread(boost::bind(&Frontend::exec, frontend, &argc, argv));
    guiThread.join();
    planThread.join();

    return EXIT_SUCCESS;
}
//==============================================================================
void PlanningProblem::plan(int* argcp, char** argv)
{
    if (!cfg->loadData) {
        std::cout << "Planning time is set to " << cfg->planningTime << " sec\n";
        if (manipulator->plan()) {
            manipulator->recordSolution();
            manipulator->store(cfg->loadDataFile.c_str());
        }
    }
    else {
        OMPL_INFORM("Loading the tree from file %s", cfg->loadDataFile.c_str());
        manipulator->load(cfg->loadDataFile.c_str());
    }
    cfg->planningDone = true;

    while(!cfg->dynamicObstacle){
        ;//std::cout << "wait for dynamic replanning" << std::endl;
    }

    std::cout << "dynamic replanning was initiated" << std::endl;
    manipulator->spawnDynamicObstacles();
    manipulator->replan();
    cfg->dynamicReplanning = true;

    while(true);
    return;
}
