#include "solutionpath.h"

SolutionPath::SolutionPath()
{
}

void SolutionPath::set(const og::PathGeometric& p, const ob::SpaceInformationPtr& si, const dd::SkeletonPtr& robot)
{
    if (p.getStateCount() == 0) {
        // TODO here some error report
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
                                   Eigen::Vector3d(0.3, 0.6, 0.9), 0.01,
                                   Drawable::DrawableType::SPHERE,
                                   Drawable::DrawableVisibility::VISIBLE);
        dc_.add(d);
    }
}
