#ifndef MYWINDOW_H_
#define MYWINDOW_H_

#include <boost/thread.hpp>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <dart/dart.h>

#include <boost/chrono/thread_clock.hpp>

#include "guimisc.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;

class MyWindow : public dart::gui::SimWindow
{
public:
    MyWindow();

    virtual ~MyWindow();

    // Documentation inherited
    virtual void timeStepping();

    // Documentation inherited
    virtual void drawSkels();

    // Documentation inherited
    virtual void keyboard(unsigned char _key, int _x, int _y);

    void setMotion(og::PathGeometric *motion);

    void initDrawTree();
    void updateDrawTree();
    void drawTree();
    void drawGhostManipulator();
    void drawManipulatorState(int state);

    Eigen::Vector3d getVertex(const ob::PlannerDataVertex &vertex);

    og::SimpleSetupPtr ss_;

private:
    og::PathGeometric *motion_ = NULL;

    int motionStep;
    int treeState;

    std::vector<Eigen::Vector3d> endEffectorPosition;
    std::vector<Eigen::Vector3d> endEffectorPositionDetached;
    std::vector<Eigen::Vector3d> solutionPositions;

    std::vector< std::vector<Eigen::Vector3d> > edges;
    bool ghostDrawn;

    dart::common::Timer timer1;
    dart::common::Timer timer2;
};

#endif // MYWINDOW_H_
