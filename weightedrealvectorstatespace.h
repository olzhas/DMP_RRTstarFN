#ifndef WEIGHTEDREALVECTORSPACE_H
#define WEIGHTEDREALVECTORSPACE_H

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>

namespace ompl
{
namespace base
{
class WeightedRealVectorStateSpace : public RealVectorStateSpace
{
public:

    double distance(const State *state1, const State *state2) const override;
};
}
}
#endif // WEIGHTEDREALVECTORSPACE_H
