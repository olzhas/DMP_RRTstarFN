#include "planningproblem.h"

int main(int argc, char* argv[])
{
    int res = system("date");
    PlanningProblem pp;
    return pp.solve(argc, argv);
}
