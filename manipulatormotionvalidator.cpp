#include "manipulatormotionvalidator.h"


//==============================================================================
bool ManipulatorMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2) const
{
    ob::State* s3;
    if (si_->isValid(s1) == false || si_->isValid(s2) == false) {
        //OMPL_WARN("Hey, the initial or final state is invalid");
        return false;
    }
#define INTERP_STEP 0.05
    for (double step = INTERP_STEP; step < 1.0; step += INTERP_STEP) {
        stateSpace_->as<ob::RealVectorStateSpace>()->interpolate(s1, s2, step, s3);
        if (si_->isValid(s3) == false) {
            //OMPL_WARN("Hey intermediate state is invalid");
            return false;
        }
    }

    return true;
}

//==============================================================================
// TODO implement motion validator
bool ManipulatorMotionValidator::checkMotion(const ob::State* s1,
                                             const ob::State* s2,
                                             std::pair<ob::State*, double>& lastValid) const
{
    OMPL_ERROR("call of the method");
    return false;
}

void ManipulatorMotionValidator::defaultSettings()
{
    stateSpace_ = si_->getStateSpace();
    if (!stateSpace_)
        throw ompl::Exception("No state space for motion validator");
}