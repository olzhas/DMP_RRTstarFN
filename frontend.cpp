#include "frontend.h"

//==============================================================================
Frontend::Frontend() { ; }
//==============================================================================
void Frontend::init()
{
    window.setWorld(pManipulator->getWorld()->clone());

}
//==============================================================================
void Frontend::setManipulator(ManipulatorPtr robot)
{
    pManipulator = robot;
    pManipulator->pWindow = std::shared_ptr<MyWindow>(&window);
}
//==============================================================================
void Frontend::exec(int* argcp, char** argv)
{
    while(!pManipulator->cfg->planningDone);
    og::PathGeometric* resultantMotion = pManipulator->getResultantMotion();

    window.setMotion(resultantMotion);
    window.ss_ = pManipulator->ss_;

    window.cfg = pManipulator->cfg;
    window.initGhostManipulators();
    glutInit(argcp, argv);
    window.initDrawTree(); // FIXME bottleneck
    window.initWindow(800, 600, "Staubli TX90XL");
    window.refreshTimer(30);
    glutMainLoop();
}
