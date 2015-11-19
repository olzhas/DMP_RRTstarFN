#include "manipulator.h"
#include "planningproblem.h"

int main(int argc, char* argv[])
{
    PlanningProblem pp;
    return pp.solve(argc, argv);
}
