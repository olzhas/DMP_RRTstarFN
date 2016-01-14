#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <fstream>
#include <yaml-cpp/yaml.h>

class Angle {
    double rad_;
    double deg_;

public:
    Angle()
        : rad_(0)
        , deg_(0)
    {
        ;
    }
    // getters
    double getDegrees() { return deg_; }
    double getRadians() { return rad_; }

    // setters
    void setDegrees(double deg)
    {
        deg_ = deg;
        rad_ = deg_ / 180.0 * M_PI;
    }
    void setRadians(double rad)
    {
        rad_ = rad;
        deg_ = rad_ / M_PI * 180.0;
    }
};

class Configuration {
private:
    YAML::Node config;

public:
    int planningTime; // time in seconds
    double dynamicPlanningTime;
    double goalBias; // biasing in goal
    double orphanedBias;
    int maxNumberNodes; // maximum number of nodes
    int pathNodes; // number of nodes for interpolation

    double rangeDeg;
    double rangeRad;
    bool dynamicReplanning;
    int cnt;
    std::vector<double> startState;
    std::vector<double> goalState;

    bool loadData;
    std::string loadDataFile;

    bool dynamicObstacle;
    bool planningDone;

    bool drawTree;
    bool drawTreeEdges;
    bool interpolate;

    int numObstacle;

    std::vector<double> dynamicObstaclePosition;

    bool* pathCollisionMap;
    size_t pathCollisionMapSize;

    uint32_t randomSeed;

    double motionCheckAccuracy;

    /** \brief the radius of sampling around orphaned nodes*/
    Angle orphanedSampleRadius;

    //==============================================================================

    Configuration();

    void readFile();
    void defaults();
};

typedef std::shared_ptr<Configuration> ConfigurationPtr;
#endif // CONFIGURATION_H
