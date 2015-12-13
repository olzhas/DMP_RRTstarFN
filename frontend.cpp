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
    while (!pManipulator->cfg->planningDone)
        ;

    pWindow->ss_ = pManipulator->ss_;

    pWindow->cfg = pManipulator->cfg;
    pWindow->initGhostManipulators();
    glutInit(argcp, argv);
    pWindow->initDrawTree(); // FIXME bottleneck

    og::PathGeometric& p = pManipulator->ss_->getSolutionPath();
    /*
    SolutionPath sp;
    sp.set(p, pManipulator->ss_->getSpaceInformation(),
           pManipulator->staubli_,
           Eigen::Vector3d(0.9, 0.6, 0.3), 0.025);
    pWindow->drawables.push_back(&sp.getDrawables());
*/
    p.interpolate(9000);
    SolutionPath spInterp;
    spInterp.set(p, pManipulator->ss_->getSpaceInformation(),
                 pManipulator->staubli_);
    pWindow->drawables.push_back(&spInterp.getDrawables());

    pWindow->initWindow(1280, 800, "Staubli TX90XL");
    pWindow->refreshTimer(10);
    glutMainLoop();
}
//==============================================================================
void Frontend::loop()
{
    pWindow->drawSkels();
}
