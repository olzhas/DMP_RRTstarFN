#ifndef MYWINDOW_H_
#define MYWINDOW_H_

#include <boost/thread.hpp>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <dart/dart.h>

#include "guimisc.h"
#include "configuration.h"

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

    void setMotion(og::PathGeometric* motion);

    void initDrawTree();
    void updateDrawTree();
    void drawTree();
    void drawManipulatorState(int state);
    void initGhostManipulators();
    void setSkeletonCollidable(dart::dynamics::SkeletonPtr& sk, const bool& isCollidable);
    void setSkeletonRGBA(dart::dynamics::SkeletonPtr& sk, const Eigen::Vector4d& _color);
    void setSkeletonAlpha(dart::dynamics::SkeletonPtr& sk, const double& alpha);

    Eigen::Vector3d getVertex(const ob::PlannerDataVertex& vertex);

    og::SimpleSetupPtr ss_;
    ConfigurationPtr cfg;

private:
    og::PathGeometric* motion_ = NULL;

    int motionStep;
    int treeState;

    class Node {
        Eigen::Vector3d position;
    public:

        Node(Eigen::Vector3d value){
            position = value;
        }

        Node(){ dtwarn << "null constructor\n";}

        double x(){ return position[0]; }
        double y(){ return position[1]; }
        double z(){ return position[2]; }

        std::vector<int> child;
    };

    std::vector<Node> endEffectorPosition;
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

typedef struct {
    double r;
    double g;
    double b;
} SimpleRGB;

#endif // MYWINDOW_H_
