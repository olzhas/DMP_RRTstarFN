#include "planningproblem.h"

int main(int argc, char* argv[])
{
    if(system("date")<0){
        printf("error with date");
    }
    PlanningProblem pp;
    return pp.solve(argc, argv);
}
