#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <fstream>
#include <yaml-cpp/yaml.h>

class Configuration
{
private:
    YAML::Node config;

public:

    int planningTime;   // time in seconds
    double goalBias;    // biasing in goal
    int maxNumberNodes; // maximum number of nodes
    int pathNodes;      // number of nodes for interpolation

    double rangeDeg;
    double rangeRad;
    bool dynamicReplanning;
    std::vector<double> startState;
    std::vector<double> goalState;

    bool loadData;
    std::string loadDataFile;

//==============================================================================

    Configuration();

    void readFile();
    void defaults();

};

#endif // CONFIGURATION_H
