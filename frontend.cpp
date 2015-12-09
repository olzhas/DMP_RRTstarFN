#include "frontend.h"

//==============================================================================
Frontend::Frontend():pWindow(new MyWindow) {

    ; }
//==============================================================================
void Frontend::init()
{
    pWindow->setWorld(pManipulator->getWorld()->clone());

}
//==============================================================================
void Frontend::setManipulator(ManipulatorPtr& robot)
{
    pManipulator = robot;
    pManipulator->pWindow = pWindow;
}
//==============================================================================
void Frontend::exec(int* argcp, char** argv)
{
    while(!pManipulator->cfg->planningDone);

    pWindow->ss_ = pManipulator->ss_;

    pWindow->cfg = pManipulator->cfg;
    pWindow->initGhostManipulators();
    glutInit(argcp, argv);
    pWindow->initDrawTree(); // FIXME bottleneck
    pWindow->initWindow(1280, 800, "Staubli TX90XL");
    pWindow->refreshTimer(30);
    glutMainLoop();
}
//==============================================================================
void Frontend::loop()
{
    pWindow->drawSkels();

}

