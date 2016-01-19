#ifndef MANIPULATORMOTIONVALIDATOR_H
#define MANIPULATORMOTIONVALIDATOR_H

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "configuration.h"
#include "DRRTstarFN.h"

namespace ob = ompl::base;

class ManipulatorMotionValidator : public ob::MotionValidator {

public:
    ManipulatorMotionValidator(ob::SpaceInformation* si)
        : MotionValidator(si),
          cfg_(new Configuration)
    {
        cfg_->readFile(); // FIXME terrible hack
        defaultSettings();
    }
    ManipulatorMotionValidator(const ob::SpaceInformationPtr& si)
        : MotionValidator(si),
          cfg_(new Configuration)
    {
        cfg_->readFile(); // FIXME terrible hack
        defaultSettings();
    }

    ManipulatorMotionValidator(const ob::SpaceInformationPtr& si, const ConfigurationPtr& cfg):
        MotionValidator(si),
        cfg_(cfg)
    {
        defaultSettings();
    }

    virtual ~ManipulatorMotionValidator()
    {
    }
    virtual bool checkMotion(const ob::State* s1, const ob::State* s2) const;
    virtual bool checkMotion(const ob::State* s1, const ob::State* s2, std::pair<ob::State*, double>& lastValid) const;

    ConfigurationPtr cfg_;

private:
    ob::StateSpacePtr stateSpace_;
    void defaultSettings();
};

#endif // MANIPULATORMOTIONVALIDATOR_H
