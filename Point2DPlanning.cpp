#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstarFN.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <dart/dart.h>

#include <ompl/config.h>
#include "config/config2D.h"

#include <boost/filesystem.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace dd = dart::dynamics;
namespace ds = dart::simulation;

class Plane2DEnvironment {
public:
    Plane2DEnvironment()
    {
        maxWidth_ = 20.0;
        maxHeight_ = 20.0;

        ob::RealVectorStateSpace* space = new ob::RealVectorStateSpace();
        space->addDimension(0.0, maxWidth_);
        space->addDimension(0.0, maxWidth_);

        ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));

        // set state validity checking for this space
        ss_->setStateValidityChecker(boost::bind(&Plane2DEnvironment::isStateValid, this, _1));
        space->setup();
        //ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
        ss_->setPlanner(ob::PlannerPtr(new og::RRTstar(ss_->getSpaceInformation())));
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col)
    {
        if (!ss_)
            return false;
        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = start_row;
        start[1] = start_col;
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = goal_row;
        goal[1] = goal_col;
        ss_->setStartAndGoalStates(start, goal);
        // generate a few solutions; all will be added to the goal;
        for (int i = 0; i < 10; ++i) {
            if (ss_->getPlanner())
                ss_->getPlanner()->clear();
            ss_->solve();
        }
        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath()) {
            ss_->simplifySolution();
            og::PathGeometric& p = ss_->getSolutionPath();
            ss_->getPathSimplifier()->simplifyMax(p);
            ss_->getPathSimplifier()->smoothBSpline(p);
            return true;
        }
        else
            return false;
    }

    void recordSolution()
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric& p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0; i < p.getStateCount(); ++i) {
            // const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            //const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            //ompl::PPM::Color& c = ppm_.getPixel(h, w);
            //c.red = 255;
            //c.green = 0;
            //c.blue = 0;
        }
    }

    void save(const char* filename)
    {
        if (!ss_)
            return;
        //ppm_.saveFile(filename);
    }

private:
    bool isStateValid(const ob::State* state) const
    {
        //const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        //const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);

        //const ompl::PPM::Color& c = ppm_.getPixel(h, w);
        //return c.red > 127 && c.green > 127 && c.blue > 127;
    }

    og::SimpleSetupPtr ss_;
    double maxWidth_;
    double maxHeight_;
};

const double default_ground_width = 2;
const double default_wall_thickness = 0.1;

dd::SkeletonPtr createGround()
{
    dd::SkeletonPtr ground = dd::Skeleton::create("ground");

    dd::BodyNode* bn = ground->createJointAndBodyNodePair<dd::WeldJoint>().second;

    std::shared_ptr<dd::BoxShape> shape = std::make_shared<dd::BoxShape>(
        Eigen::Vector3d(default_ground_width*5, default_ground_width*5,
            default_wall_thickness));
    shape->setColor(Eigen::Vector3d(1.0, 1.0, 1.0));

    bn->addCollisionShape(shape);
    bn->addVisualizationShape(shape);
    Eigen::Vector6d positions(Eigen::Vector6d::Zero());
    ground->getJoint(0)->setPositions(positions);

    return ground;
}

class Model {
    static constexpr const char* WORLD_FILE_NAME = "data/2d-problem/model.sdf";

    //    class Angle{
    //        double deg_;
    //        double rad_;

    //        Angle(): deg_(0), rad_(0) {; }

    //        void setDegrees(double deg) { deg_ = deg; rad_ = deg / 180.0 * M_PI; }
    //        void setRadians(double rad) { rad_ = rad; deg_ = rad / M_PI * 180.0; }

    //        double degrees() { return deg_; }
    //        double radians() { return rad_; }
    //    };

    class Point {
        double x_;
        double y_;

    public:
        Point() { ; }
        Point(const Point& p)
        {
            x_ = p.x();
            y_ = p.y();
        }
        Point(const double& x, const double& y)
        {
            x_ = x;
            y_ = y;
        }

        Point& operator=(Point p)
        {
            x_ = p.x();
            y_ = p.y();
            return *this;
        }

        //getters
        double x() const { return x_; }
        double y() const { return y_; }

        Eigen::Vector2d toVector() const
        {
            Eigen::Vector2d out;
            out << x_, y_;
            return out;
        }
    };
    class Line {
        Point head_;
        Point tail_;

    public:
        Line(Point head, Point tail)
            : head_(head)
            , tail_(tail)
        {
            ;
        }
        Line()
            : head_(Point(0.0, 0.0))
            , tail_(Point(0.0, 0.0))
        {
            ;
        }

        ~Line()
        {
            ;
        }

        // getters
        Point getHead() const { return head_; }
        Point getTail() const { return tail_; }

        double getLength() const
        {
            auto x = head_.x() - tail_.x();
            auto y = head_.y() - tail_.y();

            return sqrt(x * x + y * y);
        }

        Point middle() const
        {
            Point p((head_.x() + tail_.x()) / 2.0, (head_.y() + tail_.y()) / 2.0);
            return p;
        }
    };

public:
    Model()
    {
        loadWorld();
    }

    ds::WorldPtr getWorld() const { return world_; }

private:
    void loadWorld()
    {
        world_ = std::make_shared<ds::World>();
        std::vector<Line*> map;
        //map.reserve(9);
        map.resize(10);
        map[0] = new Line(Point(0.00, 0.50), Point(0.40, 0.50));
        map[1] = new Line(Point(0.50, 0.00), Point(0.50, 0.30));
        map[2] = new Line(Point(1.40, 0.00), Point(1.40, 0.50));
        map[3] = new Line(Point(1.70, 0.40), Point(1.70, 0.70));
        map[4] = new Line(Point(1.10, 0.80), Point(2.00, 0.80));
        map[5] = new Line(Point(1.10, 0.80), Point(1.10, 1.30));
        map[6] = new Line(Point(1.30, 1.10), Point(1.80, 1.10));
        map[7] = new Line(Point(1.30, 1.70), Point(1.60, 1.70));
        map[8] = new Line(Point(1.10, 1.50), Point(1.10, 1.90));
        map[9] = new Line(Point(0.20, 1.60), Point(0.70, 1.60));

        for (int i = 0; i < map.size(); ++i) {
            auto& l = map[i];

            dd::BodyNode::Properties body;
            body.mName = "box" + std::to_string(i);

            dd::ShapePtr shape(
                new dd::BoxShape(Eigen::Vector3d(l->getLength(), 0.01, 1.0)));

            body.mVizShapes.push_back(shape);
            body.mColShapes.push_back(shape);

            dd::FreeJoint::Properties properties;
            properties.mName = "box" + std::to_string(i);
            dd::SkeletonPtr box = dd::Skeleton::create("box" + std::to_string(i));
            box->createJointAndBodyNodePair<dd::FreeJoint>(nullptr, properties, body);

            Eigen::Vector6d positions(Eigen::Vector6d::Zero());
            Eigen::Vector2d middle = l->middle().toVector();

            if (l->getHead().x() == l->getTail().x()) {
                positions[2] = M_PI / 2.0;
            }

            positions[3] = middle[0];
            positions[4] = middle[1];
            box->getJoint(0)->setPositions(positions);

            world_->addSkeleton(box);
        }

        dd::SkeletonPtr ground = createGround();

        world_->addSkeleton(ground);
    }
    dart::simulation::WorldPtr world_;
};

class Window2D : public dart::gui::SimWindow {
};

int main(int argc, char** argv)
{
    Model model;

    Window2D win;
    win.setWorld(model.getWorld());

    glutInit(&argc, argv);
    win.initWindow(1280, 800, "2D demo");
    glutMainLoop();

    return 0;
}
