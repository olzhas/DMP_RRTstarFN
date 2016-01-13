#include "configuration.h"

Configuration::Configuration()
{
    defaults();
}

void Configuration::readFile()
{
    config = YAML::LoadFile("config/config.yaml");

    if (config["path-nodes"]) {
        pathNodes = config["path-nodes"].as<int>();
    }

    if (config["planning-time"]) {
        planningTime = config["planning-time"].as<int>();
    }

    if (config["max-nodes"]) {
        maxNumberNodes = config["max-nodes"].as<int>();
    }

    if (config["goal-bias"]) {
        goalBias = config["goal-bias"].as<double>();
    }

    if (config["range"]) {
        rangeDeg = config["range"].as<double>();
        rangeRad = rangeDeg / 180.0 * M_PI;
    }

    if (config["start-pos"]) {
        if(config["start-pos"].size() > 0) {
            startState.resize(config["start-pos"].size());
            for (std::size_t i(0); i < config["start-pos"].size(); ++i) {
                startState[i] = config["start-pos"][i].as<double>() / 180.0 * M_PI;
            }
        }
    }

    if (config["goal-pos"]) {
        if(config["goal-pos"].size() > 0){
            goalState.resize(config["goal-pos"].size());
            for (std::size_t i(0); i < config["goal-pos"].size(); ++i) {
                goalState[i] = config["goal-pos"][i].as<double>() / 180.0 * M_PI;
            }
        }
    }

    if (config["dynamic-replanning"]) {
        dynamicReplanning = config["dynamic-replanning"].as<bool>();
    }

    if(config["load-data"]) {
        loadData = config["load-data"].as<bool>();
    }

    if(config["load-data-file"]){
        loadDataFile = config["load-data-file"].as<std::string>();
    }

    if(config["obstacle-num"]) {
        numObstacle = config["obstacle-num"].as<int>();
    }

    if(config["dynamic-obstacle-pos"]) {

    }

    if(config["random-seed"]){
        randomSeed = config["random-seed"].as<double>();
    }

    if(config["motion-check-accuracy"]){
        motionCheckAccuracy = config["motion-check-accuracy"].as<double>();
    }
    if(config["dynamic-planning-time"]){
        dynamicPlanningTime = config["dynamic-planning-time"].as<double>();
    }

    if(config["orphaned-sample-radius-deg"]){
        orphanedSampleRadius.setDegrees(config["orphaned-sample-radius-deg"].as<double>());
    }
}

void Configuration::defaults()
{
    pathNodes = 3000;
    planningTime = 300;
    goalBias = 0.5;
    maxNumberNodes = 25000;
    loadData = false;
    loadDataFile = "mydump";
    dynamicReplanning = false;
    dynamicObstacle = false;
    planningDone = false;
    cnt = 0;
    drawTree = false;
    drawTreeEdges = false;
    interpolate = false;
    pathCollisionMap = NULL;
    randomSeed = 20;

    dynamicPlanningTime = 10.0;

    motionCheckAccuracy = 0.025;
    orphanedSampleRadius.setDegrees(30.0);
}
