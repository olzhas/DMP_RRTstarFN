#ifndef OBSTACLEMANAGER_H
#define OBSTACLEMANAGER_H

#include <yaml-cpp/yaml.h>

#include <vector>
#include <string>
#include <dart/dart.h>

#include "obstacle.h"

#define OBSTACLE_PATH "/home/olzhas/devel/staubli_dart/data/obstacles/"

namespace du = dart::utils;
namespace dd = dart::dynamics;

class ObstacleManager {
private:
    std::vector<Obstacle*> obstacles_;
    std::string configFileName_;

public:
    ObstacleManager(const std::string& configFileName = OBSTACLE_PATH "config.yaml")
        : configFileName_(configFileName)
    {
        ;
    }

    void loadAll();
};

#endif // OBSTACLEMANAGER_H
