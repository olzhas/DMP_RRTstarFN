#include "configuration.h"

Configuration::Configuration()
{
    defaults();
}

void Configuration::readFile()
{
    // TODO think about more general way to read configs
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
        startState.resize(6); // FIXME configuration value
        for (std::size_t i(0); i < config["start-pos"].size(); ++i) {
            startState[i] = config["start-pos"][i].as<double>() / 180.0 * M_PI;
        }
    }

    if (config["goal-pos"]) {
        goalState.resize(6); // FIXME configuration value
        for (std::size_t i(0); i < config["goal-pos"].size(); ++i) {
            goalState[i] = config["goal-pos"][i].as<double>() / 180.0 * M_PI;
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
}
