#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <yaml-cpp/yaml.h>
#include <dart/dart.h>

#define OBSTACLE_PATH "/home/olzhas/devel/staubli_dart/data/obstacles/config.yaml"

class Obstacle {
private:
    enum ObstacleType { WALL,
        HUMAN_BBOX,
        CUBE };

    Eigen::Vector3d pos_;
    Eigen::Vector3d rpy_;

    ObstacleType type_;

    bool dynamic_;

    dart::simulation::WorldPtr world_;

public:
    Obstacle()
        : dynamic_(false)
    {
        ;
    }

    void setPos(const Eigen::Vector3d& pos) { pos_ = pos; }
    void setRollPitchYaw(const Eigen::Vector3d& rpy) { rpy_ = rpy; }

    void setStatic() { dynamic_ = false; }
    bool isStatic() { return !dynamic_; }
    void setDynamic() { dynamic_ = true; }
    bool isDynamic() { return dynamic_; }

    static void loadObstacleArray(std::string configFile = OBSTACLE_PATH);
};

#endif // OBSTACLE_H
