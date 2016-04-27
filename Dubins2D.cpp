#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Exception.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <Eigen/Eigen>
#include <dart/common/common.h>
#include "DRRTstarFN.h"

#include <iostream>
#include <fstream>

#include <ompl/config.h>
#include "model/model.h"
#include "config/config2D.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class DubinsCarEnvironment {
public:
    double reconnectTime;

    DubinsCarEnvironment(std::string& obstacleFilepath, std::string& dynamicObstaclesFile)
        : maxWidth_(2440.0)
        , maxHeight_(2160.0)
        , prefix_("")
    {
        model_ = new Model(obstacleFilepath);
        model_->setDynamicObstaclesFile(dynamicObstaclesFile);
        // ob::StateSpacePtr space(new ob::DubinsStateSpace(0.05, true));
        ob::StateSpacePtr space(new ob::DubinsStateSpace(125, false)); // only forward
        //ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(0.11)); // only forward

        ob::RealVectorBounds bounds(2);
        bounds.setLow(0);
        bounds.high[0] = maxWidth_;
        bounds.high[1] = maxHeight_;

        space->as<ob::SE2StateSpace>()->setBounds(bounds);

        ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));
        // set state validity checking for this space
        ob::SpaceInformationPtr si = ss_->getSpaceInformation();
        model_->setSpaceInformation(si);
        ss_->setStateValidityChecker(
            boost::bind(&Model::isStateValid, model_, _1));
        space->setup();
        ss_->setPlanner(
            ob::PlannerPtr(new og::DRRTstarFN(ss_->getSpaceInformation())));
        ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(25.0);
        ss_->getPlanner()->as<og::DRRTstarFN>()->setMaxNodes(15000);
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.02);
    }

    ~DubinsCarEnvironment()
    {
        delete model_;
    }

    std::vector<ompl::base::State*> pathArray_;

    void prepareDynamic(int from)
    {
        //dart::common::Timer t1("select branch");
        try {
            ob::SpaceInformationPtr si = ss_->getSpaceInformation();
            og::PathGeometric& p = ss_->getSolutionPath();
            pathNodeCount_ = p.getStateCount();
            og::DRRTstarFN* localPlanner = ss_->getPlanner()->as<og::DRRTstarFN>();

            ompl::base::State* s = si->cloneState(p.getState(from));

            p.interpolate();
            for (size_t i = from; i < p.getStates().size(); ++i) {
                ompl::base::State* st = p.getState(i);
                pathArray_.push_back(si->cloneState(st));
            }

            localPlanner->setPreviousPath(pathArray_, from);
            //t1.start();
            localPlanner->selectBranch(s);
            //t1.stop();
            //t1.print();
            localPlanner->setSampleRadius(300);
            localPlanner->setOrphanedBias(0.400);
            localPlanner->setLocalPlanning(true);
            localPlanner->swapNN();
        }
        catch (ompl::Exception e) {
            dtwarn << "No solution, man\n";
        }
    }

    ob::State* prepareFromScratchRRTstar(const size_t from, Model::Point& final)
    {
        //dart::common::Timer t1("select branch");
        try {
            ob::SpaceInformationPtr si = ss_->getSpaceInformation();
            og::PathGeometric& p = ss_->getSolutionPath();
            pathNodeCount_ = p.getStateCount();

            ompl::base::State* s = si->cloneState(p.getState(from));

            ss_->setPlanner(
                ob::PlannerPtr(new og::DRRTstarFN(ss_->getSpaceInformation())));
            ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(25.0);
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.02);

            ob::ScopedState<> start(ss_->getStateSpace(), s);
            ob::ScopedState<> goal(ss_->getStateSpace());
            goal[0] = final.x();
            goal[1] = final.y();
            ss_->setStartAndGoalStates(start, goal);

            ss_->setup();

            ss_->getPlanner()->as<og::DRRTstarFN>()->setTerminateFirstSolution(true);

            return s;
        }
        catch (ompl::Exception e) {
            dtwarn << "No solution, man\n";
        }
    }

    ob::State* prepareFromScratchRRTstarFN(const size_t from, Model::Point& final)
    {
        //dart::common::Timer t1("select branch");
        try {
            ob::SpaceInformationPtr si = ss_->getSpaceInformation();
            og::PathGeometric& p = ss_->getSolutionPath();
            pathNodeCount_ = p.getStateCount();

            ompl::base::State* s = si->cloneState(p.getState(from));

            ss_->setPlanner(
                ob::PlannerPtr(new og::DRRTstarFN(ss_->getSpaceInformation())));
            ss_->getPlanner()->as<og::DRRTstarFN>()->setRange(25.0);
            ss_->getPlanner()->as<og::DRRTstarFN>()->setMaxNodes(4000);
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.02);

            ob::ScopedState<> start(ss_->getStateSpace(), s);
            ob::ScopedState<> goal(ss_->getStateSpace());
            goal[0] = final.x();
            goal[1] = final.y();
            ss_->setStartAndGoalStates(start, goal);

            ss_->setup();

            ss_->getPlanner()->as<og::DRRTstarFN>()->setTerminateFirstSolution(true);
            return s;
        }
        catch (ompl::Exception e) {
            dtwarn << "No solution, man\n";
        }
    }

    double removeInvalidNodes()
    {
        dart::common::Timer t1("node removal");
        t1.start();
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01);
        int removed = ss_->getPlanner()->as<og::DRRTstarFN>()->removeInvalidNodes();
        t1.stop();
        //t1.print();
        OMPL_INFORM("removed nodes from the sub tree is %d", removed);

        ss_->getProblemDefinition()->clearSolutionPaths();
        return t1.getLastElapsedTime();
    }

    void cleanup(size_t from)
    {
        ompl::base::State* s = pathArray_[from];
        ss_->getPlanner()->as<og::DRRTstarFN>()->nodeCleanUp(s);
    }

    double replan(const Model::Point& initial, const Model::Point& final,
        double time, bool clearPlanner = true)
    {
        reconnectTime = 0;
        ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.0125);
        dart::common::Timer t1("reconnect");
        t1.start();
        bool is_reconnected = ss_->getPlanner()->as<og::DRRTstarFN>()->reconnect();
        t1.stop();
        reconnectTime = t1.getLastElapsedTime();
        if (!is_reconnected) {
            ss_->solve(time);
        }
        ss_->getProblemDefinition()->clearSolutionPaths();
        // FIXME reevalute solution path without trying to solve it.
        ss_->getPlanner()->as<og::DRRTstarFN>()->evaluateSolutionPath();
        return reconnectTime + ss_->getLastPlanComputationTime();
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

        ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(0.075);

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

    bool plan(const ob::State* initial, const Model::Point& final, double time,
        bool clearPlanner = true)
    {
        if (!ss_)
            return false;
        ob::ScopedState<> start(ss_->getStateSpace(), initial);
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = final.x();
        goal[1] = final.y();
        ss_->setStartAndGoalStates(start, goal);
        // generate a few solutions; all will be added to the goal;

        ss_->getPlanner()->as<og::DRRTstarFN>()->setGoalBias(0.05);

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

        std::ofstream fout(prefix_ + fileName);
        std::ofstream foutInterp(prefix_ + filenameInterp);
        std::ofstream foutSolution(prefix_ + filenameSolution);

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

        std::ofstream ofs_v(prefix_ + fileName);
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
        std::ofstream ofs_e(prefix_ + fileName);
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
                if (space->distance(s1, s2) < 0.03) {
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

    Model* getModel() { return model_; }

    // proxy method
    void updateObstacles() { model_->updateObstacles(); }

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
    void load(const char* filename, Model::Point& i, Model::Point& f)
    {
        if (ss_->getPlanner()) {
            ss_->getPlanner()->clear();
        }

        ss_->setup();

        Model::Point initial(i);
        Model::Point final(f);

        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = initial.x();
        start[1] = initial.y();
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = final.x();
        goal[1] = final.y();

        ss_->setStartAndGoalStates(start, goal);

        ss_->getPlanner()->as<og::DRRTstarFN>()->restoreTree(filename);
    }

    size_t getPathNodesCount() const
    {
        return pathNodeCount_;
    }

    std::string statistics()
    {
        std::string output;
        output += std::to_string(ss_->getLastPlanComputationTime());

        output += ", " + std::to_string(reconnectTime);

        try {
            og::PathGeometric& p = ss_->getSolutionPath();
            output +=  ", " + std::to_string(p.length());
        }
        catch (ompl::Exception e) {
            ;
        }

        output += ", " + std::to_string(ss_->haveExactSolutionPath());
        return output;
    }

    void setRecordDirectoryPrefix(std::string path)
    {
        struct stat buff;
        if (stat(path.c_str(), &buff) != 0)
            mkdir(path.c_str(), 0777);

        prefix_ = path + "/";
    }

private:
    og::SimpleSetupPtr ss_;
    const double maxWidth_;
    const double maxHeight_;
    size_t pathNodeCount_;

    std::string prefix_;

    Model* model_;
};

std::istream& operator>>(std::istream& is, Model::Point& p)
{
    double x, y;

    is >> x;
    is >> y;
    p = Model::Point(x, y);
    return is;
}

int main(int argc, char** argv)
{
    std::string setupFilename = "config/setup.txt";
    if (argc > 1) {
        for (int i = 0; i < argc; ++i) {
            if (std::string(argv[i]) == std::string("-s") && i + 1 < argc) {
                setupFilename = argv[++i];
            }
        }
    }

    std::ifstream fin(setupFilename);

    assert(!fin.fail() && "Cannot open file");

    Model::Point start;
    Model::Point goal;
    fin >> start;
    fin >> goal;

    std::ofstream goalStartfout("goal_start.txt");
    goalStartfout << "set object 1001 circle at " << start.x() << "," << start.y() << " size 50 "
                  << "fc rgb \"#ff00ff\" front\n";
    goalStartfout << "set object 1002 circle at " << goal.x() << "," << goal.y() << " size 50 "
                  << "fc rgb \"#ff00ff\" front\n";

    goalStartfout.flush();
    double time;
    size_t iter;
    fin >> time >> iter;

    double dt = time / ((double) iter);
    size_t ITERATIONS = iter;

    std::string fileDump;
    fin >> fileDump;

    bool plan;
    fin >> plan;

    std::string obstaclesFile;
    fin >> obstaclesFile;

    std::string dynamicObstaclesFile;
    fin >> dynamicObstaclesFile;

    DubinsCarEnvironment problem(obstaclesFile, dynamicObstaclesFile);

#define PLOTTING
#ifdef PLOTTING
    if (plan) {
        for (size_t i = 0; i < ITERATIONS; i++) {
            bool clearPlanner = (i == 0);
            if (problem.plan(start, goal, dt, clearPlanner)) {
                problem.recordSolution(i);
                problem.recordTreeState(i);
                std::cout << i << " done\n";
            }
        }
        problem.store(fileDump.c_str());
    }
#endif

    std::cout << time << "\n" << dt << std::endl;
    if (system("date") < 0) {
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

    problem.updateObstacles();
    std::cout << "obstacle has moved\n";

    std::ofstream fout("benchmark_results.txt");

    fout << obstaclesFile << "\n";

    fout << "from time cost reached\n";

    const size_t benchmark_path_nodes = 9;
    //for (size_t from = 0; from < problem.getPathNodesCount(); ++from) {
    for (size_t from = 0; from < benchmark_path_nodes; ++from) {
        problem.load(fileDump.c_str(), start, goal);

        dart::common::Timer t1("benchmark");
        t1.start();

        problem.prepareDynamic(from);
        std::cout << "prepared tree for removal\n";

        //==============================================================================
        problem.recordSolution(800);
        problem.recordTreeState(800);
        std::cout << "recorded 800\n";

        double removalTime = problem.removeInvalidNodes();

        std::cout << "invalid branch removal: done\n";
        problem.recordTreeState(801);

        //==============================================================================

        const int DYNAMIC_ITERATIONS = 1;
        std::cout << std::endl;
        for (size_t i = ITERATIONS + 1; i < DYNAMIC_ITERATIONS + ITERATIONS + 1;
             i++) {
            if (problem.replan(start, goal, 600.00, false)) {
                problem.setRecordDirectoryPrefix(std::string("path_node_") + std::to_string(from));
                problem.recordSolution(i);
                problem.recordTreeState(i);
                std::cout << "done\n";
            }
        }
        t1.stop();
        fout << from << ", " << t1.getLastElapsedTime() << ", " << problem.statistics() << std::endl;
    }
    std::cout << ">>>>>>>>>>>>>>>>>>> RRT star initiated\n";
    fout << "rrt star\n";
    for (size_t from = 0; from < benchmark_path_nodes; ++from) {
        problem.load(fileDump.c_str(), start, goal);

        dart::common::Timer t1("benchmark");
        t1.start();

        ob::State *s = problem.prepareFromScratchRRTstar(from, goal);

        std::cout << "prepared tree for removal\n";

        //==============================================================================
        problem.recordSolution(800);
        problem.recordTreeState(800);
        std::cout << "recorded 800\n";
        problem.recordTreeState(801);

        //==============================================================================
        const int DYNAMIC_ITERATIONS = 1;
        std::cout << std::endl;
        for (size_t i = ITERATIONS + 1; i < DYNAMIC_ITERATIONS + ITERATIONS + 1;
             i++) {
            if (problem.plan(s, goal, 600.00, false)) {
                problem.setRecordDirectoryPrefix(std::string("rrts_path_node_") + std::to_string(from));
                problem.recordSolution(i);
                problem.recordTreeState(i);
                std::cout << "done\n";
            }
        }
        t1.stop();
        fout << from << ", " << t1.getLastElapsedTime() << ", " << problem.statistics() << std::endl;
    }
    std::cout << ">>>>>>>>>>>>>>>>>>> RRT star FN initiated\n";
    fout << "rrt star fn\n";
    for (size_t from = 0; from < benchmark_path_nodes; ++from) {
        problem.load(fileDump.c_str(), start, goal);

        dart::common::Timer t1("benchmark");
        t1.start();

        ob::State *s = problem.prepareFromScratchRRTstarFN(from, goal);
        std::cout << "prepared tree for removal\n";

        //==============================================================================
        problem.recordSolution(800);
        problem.recordTreeState(800);
        std::cout << "recorded 800\n";
        problem.recordTreeState(801);

        //==============================================================================
        const int DYNAMIC_ITERATIONS = 1;
        std::cout << std::endl;
        for (size_t i = ITERATIONS + 1; i < DYNAMIC_ITERATIONS + ITERATIONS + 1;
             i++) {
            if (problem.plan(s, goal, 600.00, false)) {
                problem.setRecordDirectoryPrefix(std::string("rrtsfn_path_node_") + std::to_string(from));
                problem.recordSolution(i);
                problem.recordTreeState(i);
                std::cout << "done\n";
            }
        }
        t1.stop();
        fout << from << ", " << t1.getLastElapsedTime() << ", " << problem.statistics() << std::endl;
    }
    return 0;
}
