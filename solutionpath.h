#ifndef SOLUTIONPATH_H
#define SOLUTIONPATH_H

#include <vector>
#include <Eigen/Eigen>
#include <ompl/base/State.h>

#include "drawable.h"

namespace ob = ompl::base;

class SolutionPath
{
private:
    DrawableCollection dc_;

    std::vector<ob::State*> states_;

    Eigen::VectorXd projection_;

public:
    SolutionPath();

    DrawableCollection& getDrawables() {
        return dc_;
    }
};

#endif // SOLUTIONPATH_H
