#ifndef WEIGHTEDREALVECTORSPACE_H
#define WEIGHTEDREALVECTORSPACE_H

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>

namespace ompl
{
namespace base
{

class WeightedRealVectorStateSampler : public RealVectorStateSampler {
public:
    WeightedRealVectorStateSampler(const StateSpace *space) : RealVectorStateSampler(space)
    {
        rng_.setLocalSeed(200390);
    }
};

class WeightedRealVectorStateSpace : public RealVectorStateSpace
{
public:
    double distance(const State *state1, const State *state2) const;
    ompl::base::StateSamplerPtr allocDefaultStateSampler() const;

    boost::function <double (const State *state1, const State *state2)> localCost;
};
}
}
#endif // WEIGHTEDREALVECTORSPACE_H
