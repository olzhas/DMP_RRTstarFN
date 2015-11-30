#include "frontend.h"

//==============================================================================
Frontend::Frontend() { ; }
//==============================================================================
void Frontend::init()
{
    window.setWorld(manipulator->getWorld());

}
//==============================================================================
void Frontend::setManipulator(ManipulatorPtr robot)
{
    manipulator = robot;
}
//==============================================================================
void Frontend::exec(int* argcp, char** argv)
{
    while(!manipulator->cfg->planningDone);
    og::PathGeometric* resultantMotion = manipulator->getResultantMotion();

    window.setMotion(resultantMotion);
    window.ss_ = manipulator->ss_;

    window.cfg = manipulator->cfg;
    window.initGhostManipulators();
    glutInit(argcp, argv);
    window.initDrawTree(); // FIXME bottleneck
    window.initWindow(800, 600, "Staubli TX90XL");
    window.refreshTimer(30);
    glutMainLoop();
}
