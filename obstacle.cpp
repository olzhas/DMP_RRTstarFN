#include "obstacle.h"

Obstacle::Obstacle()
{
}

void Obstacle::loadObstacleArray(std::string configFile)
{
    YAML::Node config = YAML::LoadFile(configFile);
}
