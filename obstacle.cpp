#include "obstacle.h"

void Obstacle::loadObstacleArray(std::string configFile)
{
    YAML::Node config = YAML::LoadFile(configFile);

    if(config["active-obstacles"]){
        if(config["active-obstacles"].size() > 0) {

        }
    } else {
        dtwarn << "Possible error";
        dtwarn << "No active obstacles";
    }
}
