#include "planningproblem.h"

PlanningProblem::PlanningProblem()
{
    cfg.readFile();
    manipulator = ManipulatorPtr(new Manipulator());
    manipulator->init(cfg);
}

int PlanningProblem::solve(int argc, char* argv[])
{
    // just to check if time ticks correctly

    boost::thread planThread(boost::bind(&PlanningProblem::plan, this, &argc, argv));

    //    if(cfg.dynamicReplanning){
    //        for(int j = 0; j < 5; j++){
    //            std::cout << "\nreplanning iteration #" << j << std::endl;
    //            manipulator->updateObstacles();
    //            manipulator->replan();
    //        }
    //    }


    planThread.join();

    frontend.setManipulator(manipulator);
    frontend.init();

    boost::thread guiThread(boost::bind(&Frontend::exec, frontend, &argc, argv));
    guiThread.join();

    return EXIT_SUCCESS;
}

void PlanningProblem::plan(int* argcp, char** argv)
{
    if (!cfg.loadData) {
        std::cout << "Planning time is set to " << cfg.planningTime << "sec\n";
        if (manipulator->plan()) {
            manipulator->recordSolution();
            manipulator->store(cfg.loadDataFile.c_str());
        }
    }
    else {
        OMPL_INFORM("Loading the tree from file %s", cfg.loadDataFile.c_str());
        manipulator->load(cfg.loadDataFile.c_str());
    }
    return;
}