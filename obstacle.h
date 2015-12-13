#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <yaml-cpp/yaml.h>
#include <dart/math/math.h>

class Obstacle
{
private:
    enum ObstacleType { WALL,
                        HUMAN_BBOX,
                        CUBE };

    Eigen::Vector3d pos_;
    Eigen::Vector3d rpy_;

    ObstacleType type_;

    bool dynamic_;

public:
    Obstacle();

    void setPos(const Eigen::Vector3d &pos) { pos_ = pos;}
    void setRollPitchYaw(const Eigen::Vector3d &rpy) { rpy_ = rpy;}

    void setStatic() { dynamic_ = false; }
    bool isStatic() { return !dynamic_; }
    void setDynamic() { dynamic_ = true; }
    bool isDynamic() {return dynamic_; }

    void loadObstacleArray(std::string configFile);
};

#endif // OBSTACLE_H
