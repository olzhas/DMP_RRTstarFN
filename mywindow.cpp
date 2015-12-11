#include "mywindow.h"

#include <fstream>
#include <ompl/geometric/planners/rrt/DRRTstarFN.h>

namespace dd = dart::dynamics;
namespace du = dart::utils;

#define SAFESPACE_DATA "/home/olzhas/devel/staubli_dart/data/"

//==============================================================================
MyWindow::MyWindow()
    : SimWindow()
    , motionStep(0)
    , treeState(0)
    , timer1("update")
    , timer2("draw")
    , cameraReset(true)
    , prevSize(0)
    , subSolutionStep(0)
{
    mZoom = 0.3;
    //mCapture = true;

}
//==============================================================================

void MyWindow::initGhostManipulators()
{
    dd::SkeletonPtr staubliStartState(du::SdfParser::readSkeleton(SAFESPACE_DATA "/safespace/model.sdf"));
    staubliStartState->setName("TX90XLHB-Start");
    for (int i = 2; i < 8; ++i) {
        staubliStartState->setPosition(i, cfg->startState[i - 2]);
    }

    setSkeletonAlpha(staubliStartState, 0.2);
    setSkeletonCollidable(staubliStartState, false);
    mWorld->addSkeleton(staubliStartState);

    //==========================================================================

    dd::SkeletonPtr staubliFinalState(du::SdfParser::readSkeleton(SAFESPACE_DATA "/safespace/model.sdf"));
    staubliFinalState->setName("TX90XLHB-Final");
    for (int i = 2; i < 8; ++i) {
        staubliFinalState->setPosition(i, cfg->goalState[i - 2]);
    }
    mWorld->addSkeleton(staubliFinalState);
    setSkeletonCollidable(staubliFinalState, false);
    setSkeletonAlpha(staubliFinalState, 0.2);
}

//==============================================================================
void MyWindow::setSkeletonCollidable(dd::SkeletonPtr& sk, const bool& isCollidable)
{
    for (size_t i = 0; i < sk->getNumBodyNodes(); ++i) {
        sk->getBodyNode(i)->setCollidable(isCollidable);
    }
}

//==============================================================================
void MyWindow::setSkeletonRGBA(dd::SkeletonPtr& sk, const Eigen::Vector4d& _color)
{
    //
    for (size_t i = 0; i < sk->getNumBodyNodes(); ++i) {
        for (size_t j = 0; j < sk->getBodyNode(i)->getNumVisualizationShapes(); ++j) {
            sk->getBodyNode(i)->getVisualizationShape(j)->setRGBA(_color);
        }
    }
}

//==============================================================================
void MyWindow::setSkeletonAlpha(dd::SkeletonPtr& sk, const double& alpha)
{
    for (size_t i = 0; i < sk->getNumBodyNodes(); ++i) {
        sk->getBodyNode(i)->getVisualizationShape(0)->setAlpha(alpha);
    }
}
//==============================================================================
MyWindow::~MyWindow()
{
}

//==============================================================================
void MyWindow::timeStepping()
{
    mWorld->step();

    //if
    //og::PathGeometric& motion_ = ss_->getSolutionPath();
    //motion_.interpolate(5000);

    //if (motion_.getStateCount() > 0) {
    bool subFlag=false;
    if (solutionStates.size() > 0) {
        double* jointSpace;
        dart::dynamics::SkeletonPtr staubli = mWorld->getSkeleton("TX90XLHB");

        if (motionStep < solutionStates.size() + subSolutionStates.size()) {

            if (cfg->pathCollisionMap != NULL ||
                    !cfg->pathCollisionMap[motionStep])
            {
                motionStep++;
                subFlag = false;
            } else {
                subSolutionStep++;
                subFlag = true;
                if(subSolutionStates.size() == subSolutionStep)
                {
                    motionStep += cfg->pathCollisionMapSize+1;
                }

            }

            for (int i = 2; i < 8; ++i) {
                if(subFlag == false){
                    staubli->setPosition(i, solutionStates[motionStep][i-2]);
                } else {
                    staubli->setPosition(i, subSolutionStates[subSolutionStep][i-2]);
                }
            }
            staubli->computeForwardKinematics(true, false, false);

            Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
            Eigen::Vector3d mytest = Eigen::Vector3d(transform.translation());

            // camera movement
            /*
            mTrans[0] = -mytest[0] * 1000.0;
            mTrans[1] = -mytest[1] * 1000.0;
            mTrans[2] = mytest[2] * 10.0;
            */
        }
        else {
            mSimulating = false;
        }
    }
}

//==============================================================================
void MyWindow::drawSkels()
{
    if (cameraReset) {
        mZoom = 0.3;
        mTrans = Eigen::Vector3d(0, -50, -1500);
        Eigen::Matrix3d mat;
        mat = Eigen::AngleAxisd(-50.0 / 180.0 * M_PI, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(4.0 / 180.0 * M_PI, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(-61.0 / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond quat(mat);
        mTrackBall.setQuaternion(quat);
    }

#ifdef DEBUG
    dtwarn << mTrans;
    Eigen::Matrix3d rotMat = mTrackBall.getRotationMatrix();
    Eigen::Vector3d angles;
    angles = dart::math::matrixToEulerXYZ(rotMat) / M_PI * 180.0;
    dtwarn << angles[0] << " " << angles[1] << " " << angles[2] << "\n";
#endif

    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
        mWorld->getSkeleton(i)->draw(mRI);

    //timer1.start();
    updateDrawTree();
    //timer1.print();
    //timer1.stop();

    //timer2.start();

    //timer2.print();
    //timer2.stop();
    drawSolutionPath();
    drawSubSolutionPath();

    for(auto it=drawables.begin(); it!= drawables.end(); ++it){
        DrawableCollection* dc = *it;
        dc->draw();
    }
}
//==============================================================================
void MyWindow::drawSolutionPath()
{
    dart::gui::SimpleRGB boxSolColor(10.0 / 255.0, 200.0 / 255.0, 200.0 / 255.0);

    GLUquadricObj* c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);

    glColor4d(boxSolColor.r, boxSolColor.g, boxSolColor.b, 0.2);
    for (int i = 0; i < solutionPositions.size(); ++i) {
        if (cfg->pathCollisionMap != NULL) {
            if (!cfg->pathCollisionMap[i])
                glColor4d(boxSolColor.r, boxSolColor.g, boxSolColor.b, 0.2);
            else if (subSolution.size() == 0)
                glColor4d(boxSolColor.r * 10, boxSolColor.g * 0.1, boxSolColor.b * 0.1, 0.2);
            else
                continue;
        }
        Eigen::Vector3d center = solutionPositions.at(i);
        glPushMatrix();
        glTranslatef(center[0], center[1], center[2]);
        glutSolidCube(0.015);
        glPopMatrix();
    }
    gluDeleteQuadric(c);
}

void MyWindow::drawSubSolutionPath()
{
    GLUquadricObj* c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);

    glColor3d(1.0, 0.2, 0.2);
    for (int i = 0; i < subSolution.size(); ++i) {
        Eigen::Vector3d center = subSolution.at(i);
        glPushMatrix();
        glTranslatef(center[0], center[1], center[2]);
        glutSolidCube(0.01);
        glPopMatrix();
    }

    gluDeleteQuadric(c);
}
//==============================================================================
void MyWindow::initDrawTree()
{
    boost::lock_guard<boost::mutex> guard(treeMutex_);
    if (!ss_ || !ss_->haveSolutionPath()) {
        std::cerr << "initDrawTree: No solution =(" << std::endl;
        // return;
    }

    // Get the planner data to visualize the vertices and the edges
    ob::PlannerData pdat(ss_->getSpaceInformation());
    ss_->getPlannerData(pdat);

    // Print the vertices to file

    dart::dynamics::SkeletonPtr staubli(mWorld->getSkeleton("TX90XLHB")->clone());

    endEffectorPosition.clear();
    endEffectorPosition.reserve(pdat.numVertices());

    for (unsigned int i(0); i < pdat.numVertices(); ++i) {
        std::vector<double> reals;
        if (pdat.getVertex(i) != ob::PlannerData::NO_VERTEX) {

            ss_->getStateSpace()->copyToReals(reals, pdat.getVertex(i).getState());

            for (size_t j(0); j < reals.size(); ++j) {
                staubli->setPosition(j + 2, reals[j]);
            }
            staubli->computeForwardKinematics(true, false, false);
            Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
            if (pdat.getVertex(i).getTag())
                endEffectorPositionDetached.push_back(transform.translation());
            else {
                Node n(transform.translation());
                n.freshness = 0.2;
                pdat.getEdges(i, n.child);
                endEffectorPosition.push_back(n);
            }
        }
    }

    if(pdat.numVertices() > 0){
        DrawableCollection* dc = new DrawableCollection(pdat.numVertices());

        dc->setCaption("initial");

        for(int i=0; i<pdat.numVertices(); ++i){
            Drawable* d = new Drawable;
            std::vector<double> reals;
            if (pdat.getVertex(i) != ob::PlannerData::NO_VERTEX) {

                ss_->getStateSpace()->copyToReals(reals, pdat.getVertex(i).getState());

                for (size_t j(0); j < reals.size(); ++j) {
                    staubli->setPosition(j + 2, reals[j]);
                }
                staubli->computeForwardKinematics(true, false, false);
                Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
                d->setPoint(transform.translation());
                d->setType(Drawable::SPHERE);
                d->setSize(0.005);
                d->setColor(Eigen::Vector3d(0.5, 0.0, 0.5));
                dc->add(d);
            }
        }
        drawables.push_back(dc);
    }

    //std::cout<<motion_ ->getStateCount() << std::endl;
    og::PathGeometric& motion_ = ss_->getSolutionPath();

    motion_.interpolate(5000);

    if (motion_.getStateCount() > 0) {
        solutionPositions.clear();
        solutionPositions.reserve(motion_.getStateCount());
        for (int j(0); j < motion_.getStateCount(); j++) {
            double* jointSpace
                    = (double*)motion_.getState(j)
                    ->as<ob::RealVectorStateSpace::StateType>()
                    ->values;

            for (int i = 2; i < 8; ++i) {
                staubli->setPosition(i, jointSpace[i - 2]);
            }
            staubli->computeForwardKinematics(true, false, false);
            Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
            solutionPositions.push_back(transform.translation());
        }
    }

    // Print the edges to file
    /*
    std::vector<unsigned int> edge_list;
    for (unsigned int i(0); i < pdat.numVertices(); ++i) {
        unsigned int n_edge = pdat.getEdges(i, edge_list);
        for (unsigned int i2(0); i2 < n_edge; ++i2) {
            std::vector<Eigen::Vector3d> temp;
            temp.push_back(getVertex(pdat.getVertex(i)));
            temp.push_back(getVertex(pdat.getVertex(edge_list[i2])));
            edges.push_back(temp);
            //printEdge(ofs_e, ss_->getStateSpace(), pdat.getVertex(i));
            //printEdge(ofs_e, ss_->getStateSpace(), pdat.getVertex(edge_list[i2]));
        }
    }
    */


}
//==============================================================================

void MyWindow::updateDrawTree()
{
    boost::lock_guard<boost::mutex> guard(treeMutex_);
    if (!ss_ || !ss_->haveSolutionPath()) {
        //std::cerr << "updateDrawTree: No solution =(" << std::endl;
        // return;
    }

    // Get the planner data to visualize the vertices and the edges
    ob::PlannerData pdat(ss_->getSpaceInformation());
    ss_->getPlannerData(pdat);

    // Print the vertices to file

    dart::dynamics::SkeletonPtr staubli(mWorld->getSkeleton("TX90XLHB")->clone());

    //endEffectorPosition.clear();
    prevSize = endEffectorPosition.size();
    endEffectorPosition.reserve(pdat.numVertices());

    std::vector<unsigned int> edge_list;
    edges.reserve(pdat.numVertices());
    //std::cout << "vertices: " << pdat.numVertices() << std::endl;

    for (unsigned int i(prevSize); i < pdat.numVertices(); ++i) {
        std::vector<double> reals;
        if (pdat.getVertex(i) != ob::PlannerData::NO_VERTEX) {
            ss_->getStateSpace()->copyToReals(reals, pdat.getVertex(i).getState());

            for (size_t j(0); j < reals.size(); ++j)
                staubli->setPosition(j + 2, reals[j]);

            staubli->computeForwardKinematics(true, false, false);
            Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
            if (pdat.getVertex(i).getTag())
                endEffectorPositionDetached.push_back(transform.translation());
            else {
                Node n(transform.translation());
                pdat.getEdges(i, n.child);
                endEffectorPosition.push_back(n);
            }

            // edges handling
            /*
            unsigned int n_edge = pdat.getEdges(i, edge_list);
            for (unsigned int i2(0); i2 < n_edge; ++i2) {
                std::vector<Eigen::Vector3d> temp;
                temp.push_back(getVertex(pdat.getVertex(i)));
                temp.push_back(getVertex(pdat.getVertex(edge_list[i2])));
                edges.push_back(temp);
            }
            */
        }
    }
    if (cfg->dynamicReplanning && cfg->cnt == 0) {
        endEffectorPositionDynamicAdded.clear();
        endEffectorPositionDetached.clear();
        cfg->cnt++;
        for (size_t i(0); i < pdat.numVertices(); ++i) {
            std::vector<double> reals;
            if (pdat.getVertex(i).getTag()) {
                ss_->getStateSpace()->copyToReals(reals, pdat.getVertex(i).getState());

                for (size_t j(0); j < reals.size(); ++j)
                    staubli->setPosition(j + 2, reals[j]);
                staubli->computeForwardKinematics(true, false, false);
                Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
                if (pdat.getVertex(i).getTag() == ompl::geometric::DRRTstarFN::NodeType::ORPHANED)
                    endEffectorPositionDetached.push_back(transform.translation());
                if (pdat.getVertex(i).getTag() == ompl::geometric::DRRTstarFN::NodeType::NEW_DYNAMIC)
                    endEffectorPositionDynamicAdded.push_back(transform.translation());
            }
        }
    }
}
//==============================================================================
Eigen::Vector3d MyWindow::getVertex(const ob::PlannerDataVertex& vertex)
{
    dart::dynamics::SkeletonPtr staubli(mWorld->getSkeleton("TX90XLHB")->clone());
    std::vector<double> reals;

    assert(vertex != ob::PlannerData::NO_VERTEX);

    ss_->getStateSpace()->copyToReals(reals, vertex.getState());
    for (size_t j(0); j < reals.size(); ++j) {
        staubli->setPosition(j + 2, reals[j]);
    }
    staubli->computeForwardKinematics(true, false, false);

    Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
    return Eigen::Vector3d(transform.translation());
}
//==============================================================================
void MyWindow::drawManipulatorState(int state)
{
    if (!ss_ || !ss_->haveSolutionPath()) {
        std::cerr << "drawManipulatorState: No solution =(" << std::endl;
    }

    // Get the planner data to visualize the vertices and the edges
    ob::PlannerData pdat(ss_->getSpaceInformation());
    ss_->getPlannerData(pdat);

    // Print the vertices to file

    dart::dynamics::SkeletonPtr staubli(mWorld->getSkeleton("TX90XLHB"));
    std::vector<double> reals;
    ss_->getStateSpace()->copyToReals(reals, pdat.getVertex(state).getState());

    for (size_t j(0); j < reals.size(); ++j) {
        staubli->setPosition(j + 2, reals[j]);
    }
    staubli->computeForwardKinematics(true, false, false);
}
//==============================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
    switch (_key) {
    case ' ': // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if (mSimulating) {
            mPlay = false;
            glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay) {
            mSimulating = false;
            glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[': // step backward
        if (!mSimulating) {
            mPlayFrame--;
            if (mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']': // step forwardward
        if (!mSimulating) {
            mPlayFrame++;
            if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case '\'':
        treeState++;
        drawManipulatorState(treeState);
        std::cout << treeState << std::endl;
        break;
    case ';':
        if (treeState >= 0)
            treeState--;
        drawManipulatorState(treeState);
        std::cout << treeState << std::endl;
        break;
    case 'g':
    case 'G':
        cameraReset = !cameraReset;
        break;
    case 'o':
        cfg->dynamicObstacle = !(cfg->dynamicObstacle);
        break;
    case 't':
        cfg->drawTree = !cfg->drawTree;
        break;
    case 'e':
        cfg->drawTreeEdges = !cfg->drawTreeEdges;
        break;
    default:
        Win3D::keyboard(_key, _x, _y);
        break;
    }

    // Keyboard control for Controller

    glutPostRedisplay();
}
