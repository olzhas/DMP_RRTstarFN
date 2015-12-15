#include "obstacle.h"

Obstacle::Obstacle(const dd::SkeletonPtr& skel, const std::string& name, bool dynamic)
    : skeleton_(skel)
    , name_(name)
    , dynamic_(dynamic)
{
    ;
}

void Obstacle::spawn()
{
    ;
}
