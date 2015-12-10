#ifndef MANIPULATORMOTIONVALIDATOR_H
#define MANIPULATORMOTIONVALIDATOR_H

#include <ompl/geometric/planners/rrt/DRRTstarFN.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "configuration.h"

namespace ob = ompl::base;

class ManipulatorMotionValidator : public ob::MotionValidator {

public:
    ManipulatorMotionValidator(ob::SpaceInformation* si)
        : MotionValidator(si),
          cfg(new Configuration)
    {
        cfg->readFile();
        defaultSettings();
    }
    ManipulatorMotionValidator(const ob::SpaceInformationPtr& si)
        : MotionValidator(si)
    {
        defaultSettings();
    }
    virtual ~ManipulatorMotionValidator()
    {
    }
    virtual bool checkMotion(const ob::State* s1, const ob::State* s2) const;
    virtual bool checkMotion(const ob::State* s1, const ob::State* s2, std::pair<ob::State*, double>& lastValid) const;

    ConfigurationPtr cfg;

private:
    ob::StateSpacePtr stateSpace_;
    void defaultSettings();
};

#endif // MANIPULATORMOTIONVALIDATOR_H
