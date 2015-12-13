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

inline void Obstacle::setStatic()
{
    dynamic = false;
}

inline bool Obstacle::isStatic()
{
    return !dynamic;
}

inline void Obstacle::setDynamic()
{
    dynamic = true;
}

inline bool Obstacle::isDynamic()
{
    return dynamic;
}

void Obstacle::loadObstacleArray(std::string configFile)
{
    YAML::Node config = YAML::LoadFile(configFile);
}
