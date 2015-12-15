#ifndef SOLUTIONPATH_H
#define SOLUTIONPATH_H

// general includes
#include <vector>

// dart includes
#include <dart/dart.h>
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

class SolutionPath {
private:
    DrawableCollection dc_;
    std::vector<ob::State*> states_;
    std::vector<Eigen::Isometry3d> poses_;

    ob::SpaceInformationPtr si_;
    std::string caption_;

    size_t step = 0;

public:
    SolutionPath() : caption_("") { ;}
    SolutionPath(std::string caption): caption_(caption) {; }

    // setters

    void set(const og::PathGeometric& p,
        const ob::SpaceInformationPtr& si,
        const dd::SkeletonPtr& robot,
        const Eigen::Vector3d& color = { 0.3, 0.6, 0.9 },
        const double& size = 0.01);

    void setCaption(const std::string& caption) { caption_ = caption; }

    // getters
    DrawableCollection& getDrawables() { return dc_; }
    std::string& getCaption() { return caption_; }

    std::vector<double>& getNextState();
};

#endif // SOLUTIONPATH_H
