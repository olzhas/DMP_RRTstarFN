#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <dart/dart.h>

namespace dd = dart::dynamics;
namespace ds = dart::simulation;

class Obstacle {
private:
    enum ObstacleType { WALL,
                        HUMAN_BBOX,
                        CUBE };

    enum ObstacleCharacteristic : bool {
        DYNAMIC,
        STATIC
    };

    Eigen::Vector3d pos_;
    Eigen::Vector3d rpy_;

    std::string name_;
    ObstacleType type_;

    bool dynamic_;

    ds::WorldPtr world_;
    dd::SkeletonPtr skeleton_;

public:
    Obstacle()
        : dynamic_(false)
    {
        ;
    }

    Obstacle(const dd::SkeletonPtr& skel, const std::string& name, bool dynamic = false);

    void setPos(const Eigen::Vector3d& pos) { pos_ = pos; }
    void setRollPitchYaw(const Eigen::Vector3d& rpy) { rpy_ = rpy; }

    void setStatic() { dynamic_ = false; }
    bool isStatic() { return !dynamic_; }
    void setDynamic() { dynamic_ = true; }
    bool isDynamic() { return dynamic_; }

    void spawn();
};

#endif // OBSTACLE_H
