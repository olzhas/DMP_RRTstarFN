#include "frontend.h"
#include "solutionpath.h"

//==============================================================================
Frontend::Frontend()
    : pWindow(new MyWindow)
{
    ;
}
//==============================================================================
void Frontend::init()
{
    pWindow->setWorld(pManipulator->getWorld()->clone());
    pManipulator->obsManager.setVizWorld(pWindow->getWorld());
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
    pWindow->ss_ = pManipulator->ss_;
    pWindow->cfg = pManipulator->cfg;
    pWindow->initGhostManipulators();
    glutInit(argcp, argv);
    pWindow->initWindow(1280, 800, "Staubli TX90XL");
    pWindow->refreshTimer(10);
    glutMainLoop();
}
//==============================================================================
void Frontend::loop()
{
    pWindow->drawSkels();
}
