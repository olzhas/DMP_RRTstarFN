#include "planningproblem.h"

//==============================================================================
PlanningProblem::PlanningProblem()
    : cfg(new Configuration)
    , manipulator(new Manipulator)
{
    cfg->readFile();

    ompl::RNG::setSeed(cfg->randomSeed);

    manipulator->init(cfg);
}
//==============================================================================
int PlanningProblem::solve(int argc, char* argv[])
{
    frontend.setManipulator(manipulator);
    frontend.init();

    boost::thread planThread(boost::bind(&PlanningProblem::plan, this, &argc, argv));
    boost::thread guiThread(boost::bind(&Frontend::exec, frontend, &argc, argv));
    boost::thread dataUpdater(boost::bind(&PlanningProblem::treeUpdate, this));
    guiThread.join();
    planThread.join();

    return EXIT_SUCCESS;
}
//==============================================================================
void PlanningProblem::plan(int* argcp, char** argv)
{
    manipulator->obsManager.spawn("wall.skel");
    manipulator->obsManager.spawn("human_box.skel");

    if (!cfg->loadData) {
        std::cout << "Planning time is set to " << cfg->planningTime << " sec\n";
        if (manipulator->plan()) {
            manipulator->recordSolution();
            manipulator->store(cfg->loadDataFile.c_str());
        }
    }
    else {
        OMPL_INFORM("Loading the tree from file %s", cfg->loadDataFile.c_str());
        manipulator->load(cfg->loadDataFile.c_str());
    }
    cfg->planningDone = true;

    while (!cfg->dynamicObstacle) {
        auto keyWait = boost::chrono::system_clock::now() + boost::chrono::milliseconds(100);
        boost::this_thread::sleep_until(keyWait);
        //std::cout << "wait for dynamic replanning" << std::endl;
    }

    std::cout << "dynamic replanning was initiated" << std::endl;
    manipulator->obsManager.spawn("cube.skel");
    manipulator->replan();
    cfg->dynamicReplanning = true;

    while (true)
        ;
    return;
}
//==============================================================================
/* this method is designed to be executed every 40ms to update the draw tree */
namespace bc = boost::chrono;
namespace ob = ompl::base;
namespace og = ompl::geometric;
void PlanningProblem::treeUpdate()
{
    // assume that eventually world will be initialized
    //while(manipulator->getWorld() == nullptr){
    auto pre_wait = bc::system_clock::now() + bc::seconds(2);
    boost::this_thread::sleep_until(pre_wait);
    //}

    DrawableCollection tree("tree");
    frontend.getWindow()->drawables.push_back(&tree);

    dart::simulation::WorldPtr pWorld(manipulator->getWorld());
    dart::dynamics::SkeletonPtr robot(pWorld->getSkeleton("TX90XLHB")->clone());
    og::SimpleSetupPtr ss_(manipulator->ss_);

    while (true) {
        auto start = bc::system_clock::now() + bc::milliseconds(100);
        ob::PlannerData pdat(ss_->getSpaceInformation());
        ss_->getPlannerData(pdat);

        if (pdat.numVertices() > 0) {
            size_t prevTreeSize = tree.size();
            size_t pdatNumVerticies = pdat.numVertices();
            for (size_t i = prevTreeSize; i < pdatNumVerticies; ++i) {
                Drawable* d = new Drawable;
                std::vector<double> reals;
                if (pdat.getVertex(i) != ob::PlannerData::NO_VERTEX) {
                    const ob::State* s = pdat.getVertex(i).getState();
                    ss_->getStateSpace()->copyToReals(reals, s);

                    for (size_t j(0); j < reals.size(); ++j) {
                        robot->setPosition(j + 2, reals[j]);
                    }
                    robot->computeForwardKinematics(true, false, false);
                    Eigen::Isometry3d transform = robot->getBodyNode("toolflange_link")->getTransform();

                    d->setPoint(transform.translation());
                    d->setType(Drawable::BOX);
                    d->setSize(0.005);
                    d->setColor(Eigen::Vector3d(0.5, 0.0, 0.5));
                    tree.add(d);
                }
            }
        }
        boost::this_thread::sleep_until(start);
    }
}
