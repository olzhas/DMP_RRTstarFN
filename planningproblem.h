#ifndef PLANNINGPROBLEM_H
#define PLANNINGPROBLEM_H

#include <memory>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// DART
#include <dart/dart.h>
#include <string>

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

    std::string fileName;
    ManipulatorPtr manipulator; //
    bool loadData;
};

#endif // PLANNINGPROBLEM_H
