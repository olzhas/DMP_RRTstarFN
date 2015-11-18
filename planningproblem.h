#ifndef PLANNINGPROBLEM_H
#define PLANNINGPROBLEM_H

#include <memory>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <dart/dart.h>
#include <string>
#include <cmath>

#include "widget.h"
#include "manipulator.h"
#include "guimisc.h"

class PlanningProblem
{
public:
    PlanningProblem();
    void configure();
    void readFile();
    int exec(int argc, char* argv[]);
    void setConfigFileName(std::string &filename);
    void plan(int* argcp, char** argv);

private:
    int planningTime;   // time in seconds
    double goalBias;    // biasing in goal
    int maxNumberNodes; // maximum number of nodes
    double range;
    std::vector<double> startState;
    std::vector<double> goalState;

    std::string fileName;
    ManipulatorPtr manipulator; //
    bool loadData;

    void setDefaultConfig();
};

#endif // PLANNINGPROBLEM_H
