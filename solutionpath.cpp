#include "solutionpath.h"

SolutionPath::SolutionPath()
{
}

void SolutionPath::set(const og::PathGeometric& p,
                       const ob::SpaceInformationPtr& si,
                       const dd::SkeletonPtr& robot,
                       Eigen::Vector3d color,
                       double size)
{
    if (p.getStateCount() == 0) {
        dtwarn << "No solution found";
        return;
    }

    for (size_t i = 0; i < p.getStateCount(); ++i) {
        ob::State* s = si->cloneState(p.getState(i));

        states_.push_back(s);

        double* jointSpace = (double*)s->as<ob::RealVectorStateSpace::StateType>()->values;
        size_t dof = si->getStateDimension();

        for (size_t j = 0; j < dof; ++j) {
            robot->setPosition(j + 2, jointSpace[j]); // FIXME hardcoded
        }
        robot->computeForwardKinematics(true, false, false);
        Eigen::Isometry3d transform = robot->getBodyNode("toolflange_link")->getTransform();
        poses_.push_back(transform);

        Drawable* d = new Drawable(transform.translation(),
                                   color, size,
                                   Drawable::DrawableType::SPHERE,
                                   Drawable::DrawableVisibility::VISIBLE);
        dc_.add(d);
    }
}
