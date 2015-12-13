#ifndef MYWINDOW_H_
#define MYWINDOW_H_

#include <boost/thread.hpp>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <dart/dart.h>

#include "configuration.h"
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

    std::vector<Eigen::Vector3d> subSolution;

    std::vector<Eigen::Vector6d> solutionStates;
    std::vector<Eigen::Vector6d> subSolutionStates;

    std::vector<DrawableCollection*> drawables;
    void initSolutionPath();
private:

    void drawSubSolutionPath();
    void drawSolutionPath();

    int motionStep;
    int subSolutionStep;
    int treeState;

    class Node {
    private:
        Eigen::Vector3d position;

    public:
        Node(Eigen::Vector3d value) : position(value), freshness(1.0) {;}

        Node() { dtwarn << "null constructor\n"; }
        ~Node()
        {
            //dtwarn << "destructor call\n";
            child.clear();
        }

        double x() { return position[0]; }
        double y() { return position[1]; }
        double z() { return position[2]; }

        std::vector<unsigned int> child;
        double freshness;

        Eigen::Vector3d getPos() { return position; }
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

    boost::mutex treeMutex_;

    //og::PathGeometric &a;


};

typedef struct {
    double r;
    double g;
    double b;
} SimpleRGB;

typedef std::shared_ptr<MyWindow> MyWindowPtr;

#endif // MYWINDOW_H_
