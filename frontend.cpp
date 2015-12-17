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
    //while (!pManipulator->cfg->planningDone)
    //    ;

    pWindow->ss_ = pManipulator->ss_;
    pWindow->cfg = pManipulator->cfg;
    pWindow->initGhostManipulators();
    glutInit(argcp, argv);

    try {
        og::PathGeometric& p = pManipulator->ss_->getSolutionPath();
        p.interpolate(2000);
        SolutionPath spInterp("main");

        spInterp.set(p, pManipulator->ss_->getSpaceInformation(),
            pManipulator->staubli_);
        pWindow->drawables.push_back(&spInterp.getDrawables());
        pWindow->solutionPaths.push_back(&spInterp);
    }
    catch (ompl::Exception e) {
        ;
    }
    /*
    SolutionPath sp;
    sp.set(p, pManipulator->ss_->getSpaceInformation(),
           pManipulator->staubli_,
           Eigen::Vector3d(0.9, 0.6, 0.3), 0.025);
    pWindow->drawables.push_back(&sp.getDrawables());
    */

    pWindow->initWindow(1280, 800, "Staubli TX90XL");
    pWindow->refreshTimer(10);
    glutMainLoop();
}
//==============================================================================
void Frontend::loop()
{
    pWindow->drawSkels();
}
