#ifndef SOLUTIONPATH_H
#define SOLUTIONPATH_H

// general includes
#include <vector>

// dart includes
#include <dart/dynamics/Skeleton.h>
#include <Eigen/Eigen>

//OMPL includes
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// local includes
#include "drawable.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dd = dart::dynamics;

class SolutionPath
{
private:
    DrawableCollection dc_;
    std::vector<ob::State*> states_;
    std::vector<Eigen::Isometry3d> poses_;

    ob::SpaceInformationPtr si_;

public:
    SolutionPath();

    void set(const og::PathGeometric &p,
             const ob::SpaceInformationPtr& si,
             const dd::SkeletonPtr& robot,
             Eigen::Vector3d color = {0.3, 0.6, 0.9},
             double size=0.01);

    DrawableCollection& getDrawables() {
        return dc_;
    }
};

#endif // SOLUTIONPATH_H
