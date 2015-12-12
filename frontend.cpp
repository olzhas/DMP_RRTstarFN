#include "frontend.h"
#include "solutionpath.h"

//==============================================================================
Frontend::Frontend() :
    pWindow(new MyWindow)
{
    ;
}
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

    //dart::dynamics::SkeletonPtr pSkel(new dart::dynamics::Skeleton);
    //pSkel->add
    while(!pManipulator->cfg->planningDone);

    pWindow->ss_ = pManipulator->ss_;

    pWindow->cfg = pManipulator->cfg;
    pWindow->initGhostManipulators();
    glutInit(argcp, argv);
    pWindow->initDrawTree(); // FIXME bottleneck

    SolutionPath sp;
    sp.set(pManipulator->ss_->getSolutionPath(),
           pManipulator->ss_->getSpaceInformation(),
           pManipulator->staubli_);
    pWindow->drawables.push_back(&sp.getDrawables());

    pWindow->initWindow(1280, 800, "Staubli TX90XL");
    pWindow->refreshTimer(10);
    glutMainLoop();
}
//==============================================================================
void Frontend::loop()
{
    pWindow->drawSkels();

}

