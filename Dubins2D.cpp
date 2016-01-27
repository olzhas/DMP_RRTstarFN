#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include "DRRTstarFN.h"
#include <dart/dart.h>

#include <mutex>
#include <thread>

#include <fstream>

#include <ompl/config.h>
#include "config/config2D.h"

#include <boost/filesystem.hpp>
//#include <boost/bind.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace dd = dart::dynamics;
namespace ds = dart::simulation;

const double default_ground_width = 2;
const double default_wall_thickness = 0.1;
const double default_radius = 0.01;

dd::SkeletonPtr createGround()
{
    dd::SkeletonPtr ground = dd::Skeleton::create("ground");

    dd::BodyNode* bn = ground->createJointAndBodyNodePair<dd::FreeJoint>().second;

    std::shared_ptr<dd::BoxShape> shape = std::make_shared<dd::BoxShape>(
        Eigen::Vector3d(default_ground_width, default_ground_width,
            default_wall_thickness));
    shape->setColor(Eigen::Vector3d(1.0, 1.0, .0));

    //bn->addCollisionShape(shape);
    bn->addVisualizationShape(shape);
    Eigen::Vector6d positions(Eigen::Vector6d::Zero());

    positions[3] = 1.0;
    positions[4] = 1.0;
    positions[5] = -5.0;

    ground->getJoint(0)->setPositions(positions);

    return ground;
}

dd::SkeletonPtr createCar()
{
    dd::SkeletonPtr car = dd::Skeleton::create("car");

    dd::BodyNode* bn = car->createJointAndBodyNodePair<dd::FreeJoint>().second;

    std::shared_ptr<dd::BoxShape> shape = std::make_shared<dd::BoxShape>(
        Eigen::Vector3d(default_radius * 5, default_radius * 3, default_radius * 2));
    shape->setColor(Eigen::Vector3d(1.0, .0, .0));

    bn->addCollisionShape(shape);
    bn->addVisualizationShape(shape);

    Eigen::Vector6d positions(Eigen::Vector6d::Zero());

    positions[3] = default_radius;
    positions[4] = default_radius;

    car->getJoint(0)->setPositions(positions);

    return car;
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
public:
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

    Model()
    {
        loadWorld();
    }

    ds::WorldPtr getWorld() const { return world_; }

    bool isStateValid(const ob::State* state)
    {
        std::lock_guard<std::mutex> guard(mutex_);

        const ob::SE2StateSpace::StateType* s = state->as<ob::SE2StateSpace::StateType>();

        if (!si_->satisfiesBounds(s))
            return false;

        double x = s->getX();
        double y = s->getY();
        double yaw = s->getYaw();

        car_->getJoint(0)->setPosition(2, yaw);
        car_->getJoint(0)->setPosition(3, x);
        car_->getJoint(0)->setPosition(4, y);

        return !world_->checkCollision();
    }

    void setSpaceInformation(ob::SpaceInformationPtr &si) { si_ = si; }

private:
    void loadWorld()
    {
        world_ = std::make_shared<ds::World>();
        world_->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));
        std::vector<Line*> map;
        //map.reserve(9);
        map.resize(10);
        map[0] = new Line(Point(0.00, 0.50), Point(0.40, 0.50));
        map[1] = new Line(Point(0.50, 0.00), Point(0.50, 0.30));
        map[2] = new Line(Point(1.40, 0.00), Point(1.40, 0.50));
        map[3] = new Line(Point(1.70, 0.40), Point(1.70, 0.70));
        map[4] = new Line(Point(1.10, 0.80), Point(2.00, 0.80));
        map[5] = new Line(Point(1.09, 0.80), Point(1.09, 1.30));
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
        car_ = createCar();

        world_->addSkeleton(ground);
        world_->addSkeleton(car_);
    }

    dart::simulation::WorldPtr world_;
    dd::SkeletonPtr car_;

    ob::SpaceInformationPtr si_;

    std::mutex mutex_;
};

class DubinsCarEnvironment {
public:
    DubinsCarEnvironment()
        : maxWidth_(2.0)
        , maxHeight_(2.0)
    {
        ob::StateSpacePtr space(new ob::DubinsStateSpace(0.1, false));

        ob::RealVectorBounds bounds(2);
        bounds.setLow(0);
        bounds.high[0] = maxWidth_;
        bounds.high[1] = maxHeight_;

        space->as<ob::SE2StateSpace>()->setBounds(bounds);

        ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));
        // set state validity checking for this space
        ob::SpaceInformationPtr si = ss_->getSpaceInformation();
        model_.setSpaceInformation(si);
        ss_->setStateValidityChecker(boost::bind(&Model::isStateValid, &model_, _1));
        space->setup();
        //ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
        ss_->setPlanner(ob::PlannerPtr(new og::DRRTstarFN(ss_->getSpaceInformation())));
        ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(0.1);
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    }

    bool plan(const Model::Point& initial, const Model::Point& final)
    {
        if (!ss_)
            return false;
        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = initial.x();
        start[1] = initial.y();
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = final.x();
        goal[1] = final.y();
        ss_->setStartAndGoalStates(start, goal);
        // generate a few solutions; all will be added to the goal;

        if (ss_->getPlanner())
            ss_->getPlanner()->clear();
        ss_->solve(60.0);

        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath()) {
            return true;
        }
        else
            return false;
    }

    void recordSolution()
    {
        const std::string fileName = "dubins-results.txt";
        std::ofstream fout(fileName);
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric& p = ss_->getSolutionPath();
        p.interpolate();
        p.printAsMatrix(fout);
    }

    void recordTreeState()
    {
        if (!ss_) {
            return;
        }
        // Get the planner data to visualize the vertices and the edges
        ob::PlannerData pdat(ss_->getSpaceInformation());
        ss_->getPlannerData(pdat);

        // Print the vertices to file
        std::ofstream ofs_v("dubins-vertices.dat");
        for (unsigned int i(0); i < pdat.numVertices(); ++i) {
            //printEdge(ofs_v, ss_->getStateSpace(), pdat.getVertex(i));
            ofs_v << std::endl;
        }

        ob::DubinsStateSpace* space;
        space = ss_->getStateSpace()->as<ob::DubinsStateSpace>();

        // Print the edges to file
        std::ofstream ofs_e("dubins-edges.dat");
        std::vector<unsigned int> edge_list;
        std::vector<double> reals;
        std::vector<double> realsOld;
        ob::State* s3 = space->allocState();
        for (unsigned int i(0); i < pdat.numVertices(); ++i) {
            unsigned int n_edge = pdat.getEdges(i, edge_list);
            const ob::State* s1 = pdat.getVertex(i).getState();
            for (unsigned int i2(0); i2 < n_edge; ++i2) {
                const ob::State* s2 = pdat.getVertex(edge_list[i2]).getState();
                const double step = 0.1;
                space->copyToReals(realsOld, s1);
                for (double t = step; t <= 1.01; t += step){

                    space->interpolate(s1, s2, t, s3);
                    space->copyToReals(reals, s3);
                    for (const auto& r : realsOld) ofs_e << r << " ";
                    realsOld = reals;
                    for (const auto& r : reals) ofs_e << r << " ";
                    ofs_e << std::endl;

                }
            }
        }
    }

    Model& getModel() { return model_; }

private:
    og::SimpleSetupPtr ss_;
    const double maxWidth_;
    const double maxHeight_;

    Model model_;
};

class Window2D : public dart::gui::SimWindow {
    ;
};

int main(int argc, char** argv)
{
    DubinsCarEnvironment problem;

    Window2D win;
    win.setWorld(problem.getModel().getWorld());

    Model::Point start(default_radius * 1.5, default_radius * 1.5);
    Model::Point goal(1.7, 1.0);

    if (problem.plan(start, goal)) {
        problem.recordSolution();
        problem.recordTreeState();
        std::cout << "done\n";
    }

    glutInit(&argc, argv);
    win.initWindow(1280, 800, "2D demo");
    glutMainLoop();
    return 0;
}