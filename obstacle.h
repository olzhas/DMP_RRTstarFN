#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <dart/math/math.h>

class Obstacle
{
private:
    enum ObstacleType { WALL,
                        HUMAN_BBOX,
                        CUBE };

    Eigen::Vector3d pos;
    Eigen::Vector3d rpy;

    ObstacleType type;

    bool dynamic;

public:
    Obstacle();

    void setPos(const Eigen::Vector3d &position);
    void setRollPitchYaw(const Eigen::Vector3d &angles);

    void setStatic();
    bool isStatic();
    void setDynamic();
    bool isDynamic();
};

#endif // OBSTACLE_H
