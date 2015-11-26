#include "frontend.h"

//==============================================================================
Frontend::Frontend() { ; }
//==============================================================================
void Frontend::init()
{
    window.setWorld(manipulator->getWorld()->clone());

}
//==============================================================================
void Frontend::setManipulator(ManipulatorPtr robot)
{
    manipulator = robot;
}
//==============================================================================
void Frontend::exec(int* argcp, char** argv)
{
    og::PathGeometric* resultantMotion = manipulator->getResultantMotion();

    window.setMotion(resultantMotion);
    window.ss_ = manipulator->ss_;

    window.cfg = ConfigurationPtr(manipulator->cfg);
    window.initGhostManipulators();
    glutInit(argcp, argv);
    window.initDrawTree();
    window.initWindow(800, 600, "Staubli TX90XL");
    window.refreshTimer(30);
    glutMainLoop();
}
