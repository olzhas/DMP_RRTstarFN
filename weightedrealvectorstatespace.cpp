#include "weightedrealvectorstatespace.h"

namespace ompl
{
namespace base
{


double ompl::base::WeightedRealVectorStateSpace::distance(const State *state1, const State *state2) const
{
    double dist = 0.0;
    const double *s1 = static_cast<const StateType*>(state1)->values;
    const double *s2 = static_cast<const StateType*>(state2)->values;

    double k = 0;
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {
        double diff = (*s1++) - (*s2++);

        switch(i){
        case 0:
        case 1:
        case 2:
            k = 10;
            break;
        default:
            k=0.01;
            break;
        }
        dist += (diff * diff) * k;
    }
    return dist;
    //return sqrt(dist);
}
}
}
