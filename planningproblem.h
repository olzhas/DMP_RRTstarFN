#ifndef PLANNINGPROBLEM_H
#define PLANNINGPROBLEM_H

#include <cmath>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <dart/dart.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>

#include "frontend.h"
#include "manipulator.h"
#include "configuration.h"
#include "solutionpath.h"

class PlanningProblem {
public:
    PlanningProblem();
    void configure();
    void readFile();
    int solve(int argc, char* argv[]);
    void plan(int* argcp, char** argv);
    void treeUpdate();

private:
    ConfigurationPtr cfg;
    ManipulatorPtr manipulator; //

    Frontend frontend;

    dart::simulation::WorldPtr demoWorld;
};

#endif // PLANNINGPROBLEM_H
