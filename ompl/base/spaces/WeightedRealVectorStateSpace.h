#ifndef WEIGHTEDREALVECTORSPACE_H
#define WEIGHTEDREALVECTORSPACE_H

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>

namespace ompl {
namespace base {

    class WeightedRealVectorStateSampler : public RealVectorStateSampler {
    public:
        WeightedRealVectorStateSampler(const StateSpace* space)
            : RealVectorStateSampler(space)
        {
            // this is the only place where I can set a random seed
            rng_.setLocalSeed(5000);
        }
    };

    class WeightedRealVectorStateSpace : public RealVectorStateSpace {
    public:
        double distance(const State* state1, const State* state2) const;
        ompl::base::StateSamplerPtr allocDefaultStateSampler() const;

        std::function<double(const State* state1, const State* state2)> localCost;
    };
}
}
#endif // WEIGHTEDREALVECTORSPACE_H
