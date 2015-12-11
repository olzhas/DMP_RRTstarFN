#include "obstacle.h"

Obstacle::Obstacle()
{

}

void Obstacle::setPos(const Eigen::Vector3d &position)
{
    pos = position;
}

void Obstacle::setRollPitchYaw(const Eigen::Vector3d &angles)
{
    rpy = angles;
}


//TODO check if inline is appropriate here
inline void Obstacle::setStatic()
{
    dynamic = false;
}

bool Obstacle::isStatic()
{
    return !dynamic;
}

void Obstacle::setDynamic()
{
    dynamic = true;
}

bool Obstacle::isDynamic()
{
    return dynamic;
}

void Obstacle::loadObstacleArray(std::string configFile)
{
    YAML::Node config = YAML::LoadFile(configFile);

}
