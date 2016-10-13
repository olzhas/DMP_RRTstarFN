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
namespace ds = dart::simulation;

class ObstacleManager {
private:
    std::vector<Obstacle*> obstacles_;
    std::string configFileName_;
    ds::WorldPtr world_;
    ds::WorldPtr vizWorld_;

public:
    ObstacleManager(const std::string& configFileName = OBSTACLE_PATH "config.yaml")
        : configFileName_(configFileName)
    {
        ;
    }

    void loadAll();
    void spawn(const std::string& name);
    void setPlanWorld(const ds::WorldPtr& world) { world_ = world; }
    void setVizWorld(const ds::WorldPtr& world) { vizWorld_ = world; }
};

#endif // OBSTACLEMANAGER_H
