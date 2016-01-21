#include "manipulatormotionvalidator.h"


//==============================================================================
bool ManipulatorMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2) const
{
    /*
    static size_t total = 0;
    static size_t too_far = 0;
    static size_t precheck = 0;
    if (total % 1000 == 0){
        OMPL_INFORM("%d %d %f, %d", too_far, total, (double)too_far / (double)total, precheck);
    }
    ++total;
    */
    if (si_->isValid(s2) == false || si_->isValid(s1) == false) {
        //OMPL_WARN("Hey, the initial or final state is invalid");
    //    ++precheck;
        return false;
    }

    ob::State* s3;
    double d = si_->distance(s1, s2);
    double interpStep = cfg_->rangeRad / d;//* 0.5;
    // fixme hardcoded
/*
    if ((1.0 / interpStep) > 2.5) {
    //    ++too_far;
        return false;
    }
*/
    interpStep = interpStep * 0.75;
    for (double step = interpStep; step < 1.0; step += interpStep) {
        stateSpace_->as<ob::RealVectorStateSpace>()->interpolate(s1, s2, step, s3);
        if (!si_->isValid(s3)) {
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
