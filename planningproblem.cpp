#include "planningproblem.h"

//==============================================================================
PlanningProblem::PlanningProblem()
    : cfg(new Configuration)
    , manipulator(new Manipulator)
{
    cfg->readFile();
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
    auto pre_wait = bc::system_clock::now() + bc::seconds(2);
    boost::this_thread::sleep_until(pre_wait);

    DrawableCollection edges("edges");
    DrawableCollection* tree = new DrawableCollection("tree");
    //DrawableCollection* treeBak;
    DrawableCollection orphans("orphans");
    frontend.getWindow()->drawables.push_back(tree);
    frontend.getWindow()->drawables.push_back(&edges);
    frontend.getWindow()->drawables.push_back(&orphans);

    dart::simulation::WorldPtr pWorld(manipulator->getWorld());
    dart::dynamics::SkeletonPtr robot(pWorld->getSkeleton("TX90XLHB")->clone());
    og::SimpleSetupPtr ss_(manipulator->ss_);

    //while(!cfg->dynamicReplanning);
    while (true) {
        auto start = bc::system_clock::now() + bc::milliseconds(20);
        ob::PlannerData pdat(ss_->getSpaceInformation());

        ss_->getPlannerData(pdat);
/*
        if (cfg->dynamicReplanning) {
            if (tree->getCaption() == "tree") {
                treeBak = tree;
                tree = new DrawableCollection("dynamic-subtree");
            }
        }
*/
        if (pdat.numVertices() > 0) {
            size_t prevTreeSize = tree->size();
            size_t pdatNumVertices = pdat.numVertices();

            /*
            if (prevTreeSize != pdatNumVertices) {
                std::cout << "not loaded yet\n";
            }
            */

            for (size_t i = prevTreeSize; i < pdatNumVertices; ++i) {
                if (pdat.getVertex(i) != ob::PlannerData::NO_VERTEX) {
                    std::vector<double> reals;
                    const ob::State* s = pdat.getVertex(i).getState();
                    ss_->getStateSpace()->copyToReals(reals, s);

                    Eigen::VectorXd currentState(8);
                    currentState << 0, 0,
                            reals[0], reals[1], reals[2],
                            reals[3], reals[4], reals[5];
                    robot->setPositions(currentState);

                    robot->computeForwardKinematics(true, false, false);
                    Eigen::Isometry3d transform = robot->getBodyNode("toolflange_link")->getTransform();
                    Eigen::Vector3d translation = transform.translation();

                    Drawable* d = new Drawable(translation,
                                               Eigen::Vector3d(translation.array().abs() / 1.750),
                                               0.005,
                                               Drawable::BOX);
                    tree->add(d);

                    if (cfg->dynamicReplanning) {
                        std::vector<unsigned int> edgeList;
                        if (pdat.getEdges(i, edgeList)) {
                            for (int j = 0; j < edgeList.size(); ++j) {
                                DrawableEdge* e = new DrawableEdge;
                                e->setStart(transform.translation());
                                const ob::State* s1 = pdat.getVertex(edgeList[j]).getState();
                                ss_->getStateSpace()->copyToReals(reals, s1);

                                for (size_t k(0); k < reals.size(); ++k) {
                                    robot->setPosition(k + 2, reals[k]);
                                }
                                robot->computeForwardKinematics(true, false, false);
                                transform = robot->getBodyNode("toolflange_link")->getTransform();
                                e->setEnd(transform.translation());
                                edges.add(e);
                            }
                        }
                        if (pdat.getIncomingEdges(i, edgeList)) {
                            for (int j = 0; j < edgeList.size(); ++j) {
                                DrawableEdge* e = new DrawableEdge;
                                e->setEnd(transform.translation());
                                const ob::State* s1 = pdat.getVertex(edgeList[j]).getState();
                                ss_->getStateSpace()->copyToReals(reals, s1);

                                for (size_t k(0); k < reals.size(); ++k) {
                                    robot->setPosition(k + 2, reals[k]);
                                }
                                robot->computeForwardKinematics(true, false, false);
                                transform = robot->getBodyNode("toolflange_link")->getTransform();
                                e->setStart(transform.translation());
                                edges.add(e);
                            }
                        }
                    }
                }
            }
            // let's assume that order does not change
            for (int i = 0; i < pdatNumVertices; ++i) {
                if (pdat.getVertex(i) != ob::PlannerData::NO_VERTEX) {
                    // ORPHANED == 1
                    if (pdat.getVertex(i).getTag() == 1) {
                        const ob::State* s = pdat.getVertex(i).getState();
                        for (size_t j = 0; j < orphans.size(); ++j) {
                            if (ss_->getSpaceInformation()->equalStates(orphans.getElement(j)->getState(), s)) {

                                std::vector<double> reals;
                                ss_->getStateSpace()->copyToReals(reals, s);

                                for (size_t j(0); j < reals.size(); ++j) {
                                    robot->setPosition(j + 2, reals[j]);
                                }
                                robot->computeForwardKinematics(true, false, false);
                                Eigen::Isometry3d transform = robot->getBodyNode("toolflange_link")->getTransform();
                                Drawable* d = new Drawable;
                                d->setPoint(transform.translation());
                                d->setType(Drawable::BOX);
                                d->setSize(0.005);
                                d->setColor(Eigen::Vector4d(0.1, 1.0, 0.1, 0.7));
                                d->setState(const_cast<ompl::base::State*>(s));
                                orphans.add(d);

                                break;
                            }
                        }
                    }
                }
            }
        }
        boost::this_thread::sleep_until(start);
    }
}
