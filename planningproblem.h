#ifndef PLANNINGPROBLEM_H
#define PLANNINGPROBLEM_H

#include <memory>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <dart/dart.h>
#include <string>
#include <cmath>

#include "frontend.h"
#include "manipulator.h"
#include "guimisc.h"
#include "configuration.h"

class PlanningProblem {
public:
    PlanningProblem();
    void configure();
    void readFile();
    int solve(int argc, char* argv[]);
    void setConfigFileName(std::string& filename);
    void plan(int* argcp, char** argv);

private:
    ManipulatorPtr manipulator; //
    Configuration cfg;

    Frontend frontend;

    dart::simulation::WorldPtr demoWorld;
};

#endif // PLANNINGPROBLEM_H
