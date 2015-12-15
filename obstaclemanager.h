#ifndef OBSTACLEMANAGER_H
#define OBSTACLEMANAGER_H

#include <vector>
#include <string>

#include "obstacle.h"

#define OBSTACLE_PATH "/home/olzhas/devel/staubli_dart/data/obstacles/config.yaml"

class ObstacleManager {
private:
    std::vector<Obstacle*> obstacles_;
    std::string configName_;

public:
    ObstacleManager(const std::string& configName = OBSTACLE_PATH)
        : configName_(configName)
    {
        ;
    }

    void loadAll();
};

#endif // OBSTACLEMANAGER_H
