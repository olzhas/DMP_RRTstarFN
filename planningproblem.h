#ifndef PLANNINGPROBLEM_H
#define PLANNINGPROBLEM_H

#include <memory>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <dart/dart.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>

#include <string>
#include <cmath>

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
