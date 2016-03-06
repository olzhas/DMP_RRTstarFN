#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include "DRRTstarFN.h"
#include <dart/dart.h>

#include <mutex>
#include <thread>

#include <iostream>
#include <fstream>
#include <tuple>

#include <ompl/config.h>
#include "config/config2D.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace dd = dart::dynamics;
namespace ds = dart::simulation;
namespace du = dart::utils;

const double default_ground_width = 2;
const double default_wall_thickness = 0.1;
const double default_radius = 0.01;
const double default_init_x = 0.25;
const double default_init_y = 0.25;

dd::SkeletonPtr createCar()
{
    dd::SkeletonPtr car = dd::Skeleton::create("car");

    dd::BodyNode* bn = car->createJointAndBodyNodePair<dd::FreeJoint>().second;

    std::shared_ptr<dd::BoxShape> shape = std::make_shared<dd::BoxShape>(Eigen::Vector3d(
        default_radius * 8, default_radius * 5, default_radius * 2));
    shape->setColor(Eigen::Vector3d(1.0, .0, .0));

    bn->addCollisionShape(shape);
    bn->addVisualizationShape(shape);

    Eigen::Vector3d positions(Eigen::Vector3d::Zero());

    positions[0] = default_init_x;
    positions[1] = default_init_y;
    positions[2] = 0;

    Eigen::Isometry3d transform1;
    transform1.setIdentity();
    // transform1.rotate(Eigen::Matrix3d::Identity());
    transform1.translate(positions);

    // dart::dynamics::FreeJoint::setTransform(car.get(), transform1);
    // car->setPositions(positions);

    return car;
}

dd::SkeletonPtr convexObstacle(const std::string& filename)
{
    // a circle with radius 0.1 and center in (0.6, 1.0)
    const std::string SAFESPACE_DATA = "/home/olzhas/devel/staubli_dart/data/";
    dd::SkeletonPtr obs = du::SkelParser::readSkeleton(SAFESPACE_DATA + filename);
    return obs;
}

class Model {
    static constexpr char WORLD_FILE_NAME[] = "data/2d-problem/model.sdf";

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

        // getters
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

    class Obstacle {

    public:
        virtual bool detectCollision(Obstacle* target)
        {
            return true;
        }
    };

    class CircularObstacle : public Obstacle {
    private:
        Eigen::Vector2d pos;
        double radius;

    public:
        CircularObstacle()
        {
            ;
        }

        bool detectCollision(Obstacle* target)
        {
            ObbObstacle* obb = static_cast<ObbObstacle*>(target);

            Eigen::Vector2d diff = (obb->getPos() - pos);
            double squareDistance = diff.dot(diff);

            if (squareDistance > radius * radius + obb->getSquareDiag()) {
                return false;
            }
            return true;
        }

        void move(const Eigen::Vector2d& p, const double& r)
        {
            pos = p;
            radius = r;
        }
    };

    class ObbObstacle : public Obstacle {
    public:
        size_t maxX, minX;
        size_t maxY, minY;

        Eigen::MatrixXd vertices;

        /** \brief constructor */
        ObbObstacle()
            : vertices(2, 4)
            , yaw(0)
            , width(0)
            , height(0)
        {
            pos << 0, 0;
            update();
        }

        /** \brief constructor */
        ObbObstacle(const Eigen::Vector2d p, double y, double w, double h)
            : vertices(2, 4)
            , yaw(y)
            , width(w)
            , height(h)
        {
            pos = p;
            calcSquareDiag();
            update();
        }

        /** \brief */
        bool detectCollision(Obstacle* target)
        {
            bool intersectX = false,
                 intersectY = false;
            ObbObstacle* obb = static_cast<ObbObstacle*>(target);
            //Eigen::Vector2d diff = pos - obb->getPos();
            //double squareDistance = diff.dot(diff);

            ObbObstacle* left = pos[0] > obb->getPos()[0] ? obb : this;
            ObbObstacle* right = pos[0] > obb->getPos()[0] ? this : obb;

            double leftX = left->vertices(0, left->maxX) - left->getPos()[0];
            double rightX = right->getPos()[0] - right->vertices(0, right->minX);

            double x = right->getPos()[0] - left->getPos()[0];

            if (x < leftX + rightX) {
                intersectX = true;
            }

            ObbObstacle* down = pos[1] > obb->getPos()[1] ? obb : this;
            ObbObstacle* up = pos[1] > obb->getPos()[1] ? this : obb;

            double downY = down->vertices(1, down->maxY) - down->getPos()[1];
            double upY = up->getPos()[1] - up->vertices(1, up->minY);

            double y = up->getPos()[1] - down->getPos()[1];

            if (y < downY + upY) {
                intersectY = true;
            }

            return intersectX & intersectY;
        }

        /** \brief */
        void calcSquareDiag() { squareDiag = width * width + height * height; }

        /** \brief */
        Eigen::Vector2d getPos() const { return pos; }

        /** \brief */
        void setPos(const Eigen::Vector2d& position)
        {
            pos = position;
            //update();
        }

        /** \brief */

        void move(const Eigen::Vector2d& position, const double y)
        {
            pos = position;
            yaw = y;
            update();
        }

        /** \brief */
        void setWidth(const double& w)
        {
            width = w;
            calcSquareDiag();
            update();
        }

        /** \brief */
        void setHeight(const double& h)
        {
            height = h;
            calcSquareDiag();
            update();
        }

        void update()
        {
            // ugly initialization
            Eigen::MatrixXd temp(2, 4);

            /*
                3---0
                |   |
                2---1
            */
            temp << width / 2.0 * cos(yaw) - height / 2.0 * sin(yaw), width / 2.0 * cos(yaw) + height / 2.0 * sin(yaw),
                -width / 2.0 * cos(yaw) + height / 2.0 * sin(yaw), -width / 2.0 * cos(yaw) - height / 2.0 * sin(yaw),
                width / 2.0 * sin(yaw) + height / 2.0 * cos(yaw), width / 2.0 * sin(yaw) - height / 2.0 * cos(yaw),
                -width / 2.0 * sin(yaw) - height / 2.0 * cos(yaw), -width / 2.0 * sin(yaw) + height / 2.0 * cos(yaw);
            vertices = temp;
            //std::cout << vertices << std::endl;

            Eigen::MatrixXd offset(2, 4);
            offset << pos[0], pos[0], pos[0], pos[0], pos[1], pos[1], pos[1], pos[1];

            vertices += offset;

            Eigen::MatrixXd::Index max[2];
            Eigen::MatrixXd::Index min[2];

            for (size_t i = 0; i < 2; ++i) {
                vertices.row(i).maxCoeff(&max[i]);
                vertices.row(i).minCoeff(&min[i]);
            }

            maxX = max[0];
            maxY = max[1];

            minX = min[0];
            minY = min[1];
        }

        double getSquareDiag() const { return squareDiag; }

    private:
        Eigen::Vector2d pos;
        double yaw;

        double width;
        double height;
        double squareDiag;
    };

    class ObstacleCollection {
    private:
        std::vector<Obstacle*> data_;

    public:
        bool detectCollision(Obstacle* ob)
        {
            for (auto& elem : data_) {
                if (elem->detectCollision(ob) == true) {
                    return true;
                }
            }
            return false;
        }

        void add(Obstacle* a) { data_.push_back(a); }
    };

    void test()
    {
        ObstacleCollection obsCollection;
        ObbObstacle car(Eigen::Vector2d(0, 0), 0, 0.08, 0.05);
        CircularObstacle circle;
        ObbObstacle wall;
    }

    Model()
    {
        //loadWorld();
        loadSimpleWorld();
    }

    void loadSimpleWorld()
    {
        std::vector<CircularObstacle*> staticCircle(2);
        staticCircle[0] = new CircularObstacle;
        staticCircle[0]->move(Eigen::Vector2d(0.5, 1.45), 0.1);

        staticCircle[1] = new CircularObstacle;
        staticCircle[1]->move(Eigen::Vector2d(0.2, 1.0), 0.15);

        for (auto& obs : staticCircle) {
            obstacles_.add(obs);
        }

        std::vector<CircularObstacle*> dynamicCircle(1);
        dynamicCircle[0] = new CircularObstacle;
        dynamicCircle[0]->move(Eigen::Vector2d(0.9, 1.05), 0.1);

        for (auto& obs : dynamicCircle) {
            obstacles_.add(obs);
        }

        const size_t numObstacles = 10;
        Eigen::MatrixXd obsCenter(numObstacles, 2);
        obsCenter << 0.20, 0.50,
            0.50, 0.15,
            1.40, 0.25,
            1.70, 0.55,
            1.55, 0.80,
            1.09, 1.05,
            1.55, 1.10,
            1.55, 1.70,
            1.10, 1.625,
            0.45, 1.60;
        bool vertical[numObstacles] = { 0, 1, 1, 1, 0, 1, 0, 0, 1, 0 };
        double widthArray[numObstacles] = {
            0.4, 0.0, 0.0, 0.0, 0.9, 0.0, 0.5, 0.3, 0.0, 0.5
        };
        double heightArray[numObstacles] = {
            0.0, 0.3, 0.5, 0.3, 0.0, 0.5, 0.0, 0.0, 0.25, 0.0
        };

        for (size_t i = 0; i < numObstacles; ++i) {
            ObbObstacle* wall = new ObbObstacle;
            if (!vertical[i]) {
                wall->setHeight(0.06);
                wall->setWidth(widthArray[i]);
            }
            else {
                wall->setHeight(heightArray[i]);
                wall->setWidth(0.06);
            }

            wall->move(Eigen::Vector2d(obsCenter(i, 0), obsCenter(i, 1)), 0);

            obstacles_.add(wall);
        }

        simpleCar_.setWidth(0.08);
        simpleCar_.setHeight(0.05);
    }

    ds::WorldPtr getWorld() const { return world_; }

    bool isStateValid(const ob::State* state)
    {
        // std::lock_guard<std::mutex> guard(mutex_);

        const ob::SE2StateSpace::StateType* s = state->as<ob::SE2StateSpace::StateType>();

        if (!si_->satisfiesBounds(s))
            return false;

        double x = s->getX();
        double y = s->getY();
        double yaw = s->getYaw();

        simpleCar_.move(Eigen::Vector2d(x, y), yaw);
        return !obstacles_.detectCollision(&simpleCar_);
        /*

        Eigen::Isometry3d transform;
        transform.setIdentity();

        Eigen::Vector3d translation;

        translation[0] = x;
        translation[1] = y;
        translation[2] = 0;

        transform.translate(translation);

        Eigen::Matrix3d m;
        m.setIdentity();
        m = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        transform.rotate(m);

        dart::dynamics::FreeJoint::setTransform(car_.get(), transform);

        bool worldCol = world_->checkCollision();

        return !worldCol;
*/
    }

    void updateObstacles()
    {
        Eigen::Isometry3d transformation;
        transformation.setIdentity();

        Eigen::Vector3d translation(Eigen::Vector3d::Zero());
        translation[0] = 1.07;
        translation[1] = 1.4;
        transformation.translate(translation);

        dart::dynamics::FreeJoint::setTransform(dynamicObstacle_.get(),
            transformation);
    }

    void setSpaceInformation(ob::SpaceInformationPtr& si) { si_ = si; }

private:
    void loadWorld()
    {
        world_ = std::make_shared<ds::World>();
        world_->setGravity(Eigen::Vector3d(Eigen::Vector3d::Zero()));
        std::vector<Line*> map;

        enum class Scenario : char { DEFAULT,
            DETOUR };
        Scenario scenario = Scenario::DEFAULT;
        switch (scenario) {
        case Scenario::DETOUR:

            break;

        default:
            // map.reserve(9);
            map.resize(10);
            map[0] = new Line(Point(0.00, 0.50), Point(0.40, 0.50));
            map[1] = new Line(Point(0.50, 0.00), Point(0.50, 0.30));
            map[2] = new Line(Point(1.40, 0.00), Point(1.40, 0.50));
            map[3] = new Line(Point(1.70, 0.40), Point(1.70, 0.70));
            map[4] = new Line(Point(1.10, 0.80), Point(2.00, 0.80));
            map[5] = new Line(Point(1.09, 0.80), Point(1.09, 1.30));
            map[6] = new Line(Point(1.30, 1.10), Point(1.80, 1.10));
            map[7] = new Line(Point(1.40, 1.70), Point(1.70, 1.70));
            map[8] = new Line(Point(1.10, 1.50), Point(1.10, 1.75));
            map[9] = new Line(Point(0.20, 1.60), Point(0.70, 1.60));
            break;
        }

        for (size_t i = 0; i < map.size(); ++i) {
            auto& l = map[i];

            dd::BodyNode::Properties body;
            body.mName = "box" + std::to_string(i);

            dd::ShapePtr shape(
                new dd::BoxShape(Eigen::Vector3d(l->getLength(), 0.02, 1.0)));

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
        dynamicObstacle_ = convexObstacle("obstacles/r1-circle.skel");
        world_->addSkeleton(dynamicObstacle_);

        car_ = createCar();
        world_->addSkeleton(car_);

        world_->addSkeleton(convexObstacle("obstacles/r15-circle.skel"));
        world_->addSkeleton(convexObstacle("obstacles/r1-circle-side.skel"));
    }

    dart::simulation::WorldPtr world_;
    dd::SkeletonPtr car_;

    dd::SkeletonPtr dynamicObstacle_;

    ob::SpaceInformationPtr si_;

    std::mutex mutex_;

    ObstacleCollection obstacles_;
    ObbObstacle simpleCar_;
};

class DubinsCarEnvironment {
public:
    DubinsCarEnvironment()
        : maxWidth_(2.440)
        , maxHeight_(2.160)
    {
        // ob::StateSpacePtr space(new ob::DubinsStateSpace(0.05, true));
        ob::StateSpacePtr space(
            new ob::DubinsStateSpace(0.11, false)); // only forward

        ob::RealVectorBounds bounds(2);
        bounds.setLow(0);
        bounds.high[0] = maxWidth_;
        bounds.high[1] = maxHeight_;

        space->as<ob::SE2StateSpace>()->setBounds(bounds);

        ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));
        // set state validity checking for this space
        ob::SpaceInformationPtr si = ss_->getSpaceInformation();
        model_.setSpaceInformation(si);
        ss_->setStateValidityChecker(
            boost::bind(&Model::isStateValid, &model_, _1));
        space->setup();
        // ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 /
        // space->getMaximumExtent());
        ss_->setPlanner(
            ob::PlannerPtr(new og::DRRTstarFN(ss_->getSpaceInformation())));
        ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(0.03);
        ss_->getPlanner()->as<og::DRRTstarFN>()->setMaxNodes(15000);
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.0125);
    }

    std::vector<ompl::base::State*> pathArray_;

    void prepareDynamic()
    {
        dart::common::Timer t1("select branch");
        try {
            ob::SpaceInformationPtr si = ss_->getSpaceInformation();
            og::PathGeometric& p = ss_->getSolutionPath();
            og::DRRTstarFN* localPlanner = ss_->getPlanner()->as<og::DRRTstarFN>();

            int from = 9;

            ompl::base::State* s = si->cloneState(p.getState(from));

            p.interpolate();
            for (size_t i = from; i < p.getStates().size(); ++i) {
                ompl::base::State* st = p.getState(i);
                pathArray_.push_back(si->cloneState(st));
            }

            localPlanner->setPreviousPath(pathArray_, from);
            t1.start();
            localPlanner->selectBranch(s);
            t1.stop();
            t1.print();
            localPlanner->setSampleRadius(0.1);
            localPlanner->setOrphanedBias(0.50);
            localPlanner->setLocalPlanning(true);
            localPlanner->swapNN();
        }
        catch (ompl::Exception e) {
            dtwarn << "No solution, man\n";
        }
    }

    void removeInvalidNodes()
    {
        dart::common::Timer t1("node removal");
        t1.start();
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01);
        int removed = ss_->getPlanner()->as<og::DRRTstarFN>()->removeInvalidNodes();
        t1.stop();
        t1.print();
        OMPL_INFORM("removed nodes from the sub tree is %d", removed);

        ss_->getProblemDefinition()->clearSolutionPaths();
    }

    void cleanup()
    {
        int from = 9;

        ompl::base::State* s = pathArray_[from];
        ss_->getPlanner()->as<og::DRRTstarFN>()->nodeCleanUp(s);
        // ss_->getProblemDefinition()->clearSolutionPaths();
        // ss_->solve(0.010);
    }

    bool replan(const Model::Point& initial, const Model::Point& final,
        double time, bool clearPlanner = true)
    {
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.0125);

        // ss_->getPlanner()->as<og::DRRTstarFN>()->reconnect();
        ss_->solve(time);

        // REGRESSION
        // ss_->getPlanner()->as<og::DRRTstarFN>()->nodeCleanUp(s);
        ss_->getProblemDefinition()->clearSolutionPaths();
        // FIXME reevalute solution path without trying to solve it.
        ss_->getPlanner()->as<og::DRRTstarFN>()->evaluateSolutionPath();
        return true;
    }

    bool plan(const Model::Point& initial, const Model::Point& final, double time,
        bool clearPlanner = true)
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

        ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(0.1);

        if (ss_->getPlanner())
            if (clearPlanner)
                ss_->getPlanner()->clear();

        ss_->solve(time);

        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath()) {
            return true;
        }
        else
            return false;
    }

    void recordSolution() { recordSolution(-1); }

    void recordSolution(int num)
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;

        std::string fileName;
        std::string filenameInterp;
        std::string filenameSolution;

        if (num != -1) {
            fileName = "dubins-results" + std::to_string(num) + ".txt";
            filenameInterp = "dubins-results-interp" + std::to_string(num) + ".txt";
            filenameSolution = "dubins-results-is_solution" + std::to_string(num) + ".txt";
        }
        else {
            fileName = "dubins-results.txt";
            filenameInterp = "dubins-results-interp.txt";
            filenameSolution = "dubins-results-is_solution.txt";
        }

        std::ofstream fout(fileName);
        std::ofstream foutInterp(filenameInterp);
        std::ofstream foutSolution(filenameSolution);

        foutSolution << ss_->haveExactSolutionPath() << std::endl;

        og::PathGeometric& p = ss_->getSolutionPath();
        p.printAsMatrix(fout);
        p.interpolate();
        p.printAsMatrix(foutInterp);
    }

    //==============================================================================
    void printEdge(std::ostream& os, const ob::StateSpacePtr& space,
        const ob::PlannerDataVertex& vertex)
    {
        std::vector<double> reals;
        if (vertex != ob::PlannerData::NO_VERTEX) {
            space->copyToReals(reals, vertex.getState());
            for (size_t j(0); j < reals.size(); ++j)
                os << " " << reals[j];
            os << " " << vertex.getTag();
        }
    }

    void recordTreeState() { recordTreeState(-1); }

    void recordTreeState(int num)
    {
        if (!ss_) {
            return;
        }
        // Get the planner data to visualize the vertices and the edges
        ob::PlannerData pdat(ss_->getSpaceInformation());
        ss_->getPlannerData(pdat);

        // Print the vertices to file
        std::string fileName;
        if (num == -1)
            fileName = "dubins-vertices.dat";
        else
            fileName = "dubins-vertices" + std::to_string(num) + ".dat";

        std::ofstream ofs_v(fileName);
        for (unsigned int i(0); i < pdat.numVertices(); ++i) {
            printEdge(ofs_v, ss_->getStateSpace(), pdat.getVertex(i));
            ofs_v << std::endl;
        }

        ob::DubinsStateSpace* space;
        space = ss_->getStateSpace()->as<ob::DubinsStateSpace>();

        // Print the edges to file
        if (num == -1)
            fileName = "dubins-edges.dat";
        else
            fileName = "dubins-edges" + std::to_string(num) + ".dat";
        std::ofstream ofs_e(fileName);
        std::vector<unsigned int> edge_list;
        std::vector<double> reals;
        std::vector<double> realsOld;
        bool isMajorTree = false;
        ob::State* s3 = space->allocState();
        for (unsigned int i(0); i < pdat.numVertices(); ++i) {
            unsigned int n_edge = pdat.getEdges(i, edge_list);
            const ob::State* s1 = pdat.getVertex(i).getState();
            isMajorTree = pdat.getVertex(i).getTag();
            for (unsigned int i2(0); i2 < n_edge; ++i2) {
                const ob::State* s2 = pdat.getVertex(edge_list[i2]).getState();
                double step = 0.05;
                if (space->distance(s1, s2) < 0.1) {
                    step = 0.2;
                }
                space->copyToReals(realsOld, s1);
                for (double t = step; t <= 1.01; t += step) {
                    space->interpolate(s1, s2, t, s3);
                    space->copyToReals(reals, s3);
                    for (const auto& r : realsOld)
                        ofs_e << r << " ";
                    realsOld = reals;
                    for (const auto& r : reals)
                        ofs_e << r << " ";
                    //
                    ofs_e << "0x" << std::hex << (isMajorTree ? 0x4488AA : 0xDD6060)
                          << std::endl;
                }
            }
        }
    }

    Model& getModel() { return model_; }

    // proxy method
    void updateObstacles() { model_.updateObstacles(); }

    //==============================================================================
    void store(const char* filename)
    {
        // Get the planner data to visualize the vertices and the edges
        ob::PlannerData pdat(ss_->getSpaceInformation());
        ss_->getPlannerData(pdat);

        ob::PlannerDataStorage pdstorage;

        pdstorage.store(pdat, filename);
    }

    //==============================================================================
    void load(const char* filename)
    {
        if (ss_->getPlanner()) {
            ss_->getPlanner()->clear();
        }

        ss_->setup();

        // FIXME
        Model::Point initial(default_radius * 25, default_radius * 25);
        Model::Point final(1.7, 1.0);

        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = initial.x();
        start[1] = initial.y();
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = final.x();
        goal[1] = final.y();

        ss_->setStartAndGoalStates(start, goal);

        ss_->getPlanner()->as<og::DRRTstarFN>()->restoreTree(filename);
    }

private:
    og::SimpleSetupPtr ss_;
    const double maxWidth_;
    const double maxHeight_;

    Model model_;
};

class Window2D : public dart::gui::SimWindow {
    void drawSkels()
    {
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        // glMatrixMode(GL_PROJECTION);
        dart::gui::SimWindow::drawSkels();
    }
};

int main(int argc, char** argv)
{
    DubinsCarEnvironment problem;

    Model::Point start(default_radius * 25, default_radius * 25);
    Model::Point goal(1.7, 1.0);

    const double time = 600.0;
    const double dt = 2.5;
    const int ITERATIONS = time / dt;

    std::string fileDump = "dubins5.dump";
    bool plan = true;

#define PLOTTING
#ifdef PLOTTING
    if (plan) {
        for (int i = 0; i < ITERATIONS; i++) {
            bool clearPlanner = (i == 0);
            if (problem.plan(start, goal, dt, clearPlanner)) {
                problem.recordSolution(i);
                problem.recordTreeState(i);
                std::cout << i << " done\n";
            }
        }
        problem.store(fileDump.c_str());
    }
    else {
        problem.load(fileDump.c_str());
    }
#endif

    std::cout << time << "\n" << dt << std::endl;
    if (!system("date")) {
        std::cout << "cannot run system()\n";
    }

#ifdef SOLVING
    if (plan) {
        if (problem.plan(start, goal, time)) {
            problem.recordSolution();
            problem.recordTreeState();
            std::cout << "done\n";
        }
        problem.store(fileDump.c_str());
    }
    else {
        problem.load(fileDump.c_str());
    }
#endif

    // for (int i = 0; i < 4; ++i) {
    problem.updateObstacles();
    //}
    std::cout << "obstacle has moved\n";

    problem.prepareDynamic();

    std::cout << "prepared tree for removal\n";

    //==============================================================================
    problem.recordSolution(800);
    problem.recordTreeState(800);
    std::cout << "recorded 800\n";

    problem.removeInvalidNodes();

    std::cout << "invalid branch removal: done\n";
    problem.recordTreeState(801);

    //==============================================================================

    const int DYNAMIC_ITERATIONS = 1;
    std::cout << std::endl;
    for (size_t i = ITERATIONS + 1; i < DYNAMIC_ITERATIONS + ITERATIONS + 1;
         i++) {
        if (problem.replan(start, goal, 120.00, false)) {
            // problem.cleanup();
            problem.recordSolution(i);
            problem.recordTreeState(i);
            std::cout << "done\n";
        }
    }

    Window2D win;
    win.setWorld(problem.getModel().getWorld());
    glutInit(&argc, argv);
    win.initWindow(1280, 800, "2D demo");
    glutMainLoop();

    return 0;
}
