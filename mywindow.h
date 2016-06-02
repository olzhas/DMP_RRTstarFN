#ifndef MYWINDOW_H_
#define MYWINDOW_H_

#include <thread>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <dart/dart.h>

#include "configuration.h"
#include "solutionpath.h"
#include "drawable.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;

class MyWindow : public dart::gui::SimWindow {
public:
    MyWindow();

    virtual ~MyWindow();

    // Documentation inherited
    virtual void timeStepping();

    // Documentation inherited
    virtual void drawSkels();

    // Documentation inherited
    virtual void keyboard(unsigned char _key, int _x, int _y);

    void initDrawTree();
    void updateDrawTree();

    void drawManipulatorState(int state);
    void initGhostManipulators();
    void setSkeletonCollidable(dart::dynamics::SkeletonPtr& sk, const bool& isCollidable);
    void setSkeletonRGBA(dart::dynamics::SkeletonPtr& sk, const Eigen::Vector4d& _color);
    void setSkeletonAlpha(dart::dynamics::SkeletonPtr& sk, const double& alpha);

    Eigen::Vector3d getVertex(const ob::PlannerDataVertex& vertex);

    og::SimpleSetupPtr ss_;
    og::SimpleSetupPtr subSolutionSetup_;
    og::SimpleSetupPtr ssBak_;
    ConfigurationPtr cfg;

    dart::simulation::WorldPtr getWorld() { return mWorld; }
    void initSolutionPath();

    std::vector<DrawableCollection*> drawables;

    // solutionPaths[0] is active solution
    std::vector<SolutionPath*> solutionPaths;

private:
    int motionStep;
    int treeState;

    std::vector<Eigen::Vector3d> endEffectorPositionDetached;
    std::vector<Eigen::Vector3d> endEffectorPositionDynamicAdded;
    std::vector<Eigen::Vector3d> solutionPositions;

    std::vector<std::vector<Eigen::Vector3d> > edges;

    dart::common::Timer timer1;
    dart::common::Timer timer2;
    bool cameraReset;

    unsigned int prevSize;
    bool dynamicObstacle;
};

typedef std::shared_ptr<MyWindow> MyWindowPtr;

#endif // MYWINDOW_H_
