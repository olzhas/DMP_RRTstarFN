#include "planningproblem.h"

PlanningProblem::PlanningProblem()
{
    manipulator = ManipulatorPtr(new Manipulator);
    loadData = false;
    configure();
}

void PlanningProblem::configure()
{
    setDefaultConfig();
    readFile();
}

void PlanningProblem::setDefaultConfig()
{
    manipulator->setPathNodes(3000);
    manipulator->setPlanningTime(60*5);
    manipulator->setGoalBias(0.5);
    manipulator->setMaxNodes(25000);
}

void PlanningProblem::readFile()
{
    // TODO think about more general way to read configs
    YAML::Node config = YAML::LoadFile("config/config.yaml");

    if(config["path-nodes"]){

        manipulator->setPathNodes(config["path-nodes"].as<int>());
    }

    if(config["planning-time"]){
        planningTime = config["planning-time"].as<int>();
        manipulator->setPlanningTime(planningTime);
    }

    if(config["max-nodes"]) {
        maxNumberNodes = config["max-nodes"].as<int>();
        manipulator->setMaxNodes(maxNumberNodes);
    }

    if(config["goal-bias"]) {
        manipulator->setGoalBias(config["goal-bias"].as<double>());
    }

    if(config["range"]){
        range = config["range"].as<double>();
        manipulator->setRange(range / 180.0 * M_PI);
    }

    if(config["start-pos"]){
        for(std::size_t i(0);i<config["start-pos"].size(); ++i){
            startState.push_back(config["start-pos"][i].as<double>());
        }
    }

    if(config["goal-pos"]){
        for(std::size_t i(0);i<config["goal-pos"].size(); ++i){
            goalState.push_back(config["goal-pos"][i].as<double>());
        }
    }

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
