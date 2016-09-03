#include "obstacle.h"

Obstacle::Obstacle(const dd::SkeletonPtr& skel, const std::string& name,
                   bool dynamic)
    : name_(name), dynamic_(dynamic), skeleton_(skel) {
  ;
}

void Obstacle::spawn() { ; }
