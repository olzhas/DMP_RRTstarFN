#include "mywindow.h"

#include <fstream>

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
    , cameraReset(false)
{
    motion_ = NULL;
    mZoom = 0.2;
    //mCapture = true;
}
//==============================================================================

void MyWindow::initGhostManipulators()
{
    dd::SkeletonPtr staubliStartState(du::SdfParser::readSkeleton(SAFESPACE_DATA "/safespace/model.sdf"));
    staubliStartState->setName("TX90XLHB-Start");
    for (int i = 2; i < 8; ++i) {
        staubliStartState->setPosition(i, cfg.startState[i - 2]);
    }

    setSkeletonAlpha(staubliStartState, 0.2);
    setSkeletonCollidable(staubliStartState, false);
    mWorld->addSkeleton(staubliStartState);

    //==========================================================================

    dd::SkeletonPtr staubliFinalState(du::SdfParser::readSkeleton(SAFESPACE_DATA "/safespace/model.sdf"));
    staubliFinalState->setName("TX90XLHB-Final");
    for (int i = 2; i < 8; ++i) {
        staubliFinalState->setPosition(i, cfg.goalState[i - 2]);
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
        for(size_t j = 0; j < sk->getBodyNode(i)->getNumVisualizationShapes(); ++j){
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

void MyWindow::setMotion(og::PathGeometric* motion)
{
    motion_ = motion;
}

//==============================================================================
void MyWindow::timeStepping()
{
    mWorld->step();

    if (motion_ != NULL) {
        dart::dynamics::SkeletonPtr staubli = mWorld->getSkeleton("TX90XLHB");
        if (motionStep < motion_->getStateCount()) {
            //std::cout<<motion_ ->getStateCount() << std::endl;
            double* jointSpace
                    = (double*)motion_->getState(motionStep)
                    ->as<ob::RealVectorStateSpace::StateType>()
                    ->values;

            for (int i = 2; i < 8; ++i) {
                staubli->setPosition(i, jointSpace[i - 2]);
            }
            staubli->computeForwardKinematics(true, false, false);

            motionStep++;

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
    if(cameraReset){
        mTrans = Eigen::Vector3d(00, 250, -2000);
        Eigen::Matrix3d mat;
        mat = Eigen::AngleAxisd(50.0/180.0*M_PI, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(-3.5/180.0*M_PI, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(60.0/180.0*M_PI, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond quat(mat);
        mTrackBall.setQuaternion(quat);
    } else {
        dtwarn << mTrans;
        double alpha, beta, gamma;
        Eigen::Matrix3d rotMat = mTrackBall.getRotationMatrix();
        beta = asin(rotMat(0,2));
        alpha = acos(rotMat(2,2)/cos(beta));
        gamma = acos(rotMat(0,0)/cos(beta));

        dtwarn << "alpha " << alpha / M_PI * 180.0
               << " beta " << beta  / M_PI * 180.0
               << " gamma "<< gamma / M_PI * 180.0;
    }


    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
        mWorld->getSkeleton(i)->draw(mRI);

    //timer1.start();
    updateDrawTree();
    //timer1.print();
    //timer1.stop();

    //timer2.start();
    drawTree();
    //timer2.print();
    //timer2.stop();
}
//==============================================================================
void MyWindow::drawTree()
{

    dart::gui::SimpleRGB boxColor(255.0/255.0, 10.0/255.0, 0/255.0); // orange
    //dart::gui::SimpleRGB boxColor(215.0/255.0, 225.0/255.0,43.0/255.0);
    dart::gui::SimpleRGB boxDetachedColor(200.0/255.0, 0.0/255.0, 200.0/255.0);
    dart::gui::SimpleRGB boxSolColor(10.0/255.0, 200.0/255.0, 200.0/255.0);

    GLUquadricObj *c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);
    //glPushMatrix();

    glColor4d(boxColor.r, boxColor.g, boxColor.b, 0.2);
    for (int i = 0; i < endEffectorPosition.size(); ++i) {
        Eigen::Vector3d center = endEffectorPosition.at(i);
        glPushMatrix();
        glTranslatef(center[0], center[1], center[2]);
        glutSolidCube(0.01);
        glPopMatrix();
    }

    glColor3d(boxSolColor.r, boxSolColor.g, boxSolColor.b);
    for (int i = 0; i < solutionPositions.size(); ++i) {
        Eigen::Vector3d center = solutionPositions.at(i);
        glPushMatrix();
        glTranslatef(center[0], center[1], center[2]);
        glutSolidCube(0.015);
        glPopMatrix();
    }

    glColor3d(boxDetachedColor.r, boxDetachedColor.g, boxDetachedColor.b);
    for (int i = 0; i < endEffectorPositionDetached.size(); ++i) {
        Eigen::Vector3d center = endEffectorPositionDetached.at(i);
        glPushMatrix();
        glTranslatef(center[0], center[1], center[2]);
        glutSolidCube(0.025);
        glPopMatrix();
    }
    gluDeleteQuadric(c);

    /*
    for (int i = 0; i < edges.size(); ++i) {
        Eigen::Vector3d start = edges[i][0];
        Eigen::Vector3d end = edges[i][1];
        dart::gui::drawLine3D(start, end);
    }
*/
}

//==============================================================================
void MyWindow::initDrawTree()
{
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
            else
                endEffectorPosition.push_back(transform.translation());
        }
    }

    //std::cout<<motion_ ->getStateCount() << std::endl;
    if (motion_ != NULL) {
        for (int j(0); j < motion_->getStateCount(); j++) {
            double* jointSpace
                    = (double*)motion_->getState(j)
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
}
//==============================================================================

void MyWindow::updateDrawTree()
{

    if (!ss_ || !ss_->haveSolutionPath()) {
        std::cerr << "updateDrawTree: No solution =(" << std::endl;
        // return;
    }

    // Get the planner data to visualize the vertices and the edges
    ob::PlannerData pdat(ss_->getSpaceInformation());
    ss_->getPlannerData(pdat);

    // Print the vertices to file

    dart::dynamics::SkeletonPtr staubli(mWorld->getSkeleton("TX90XLHB")->clone());

    //endEffectorPosition.clear();
    unsigned int prevSize = endEffectorPosition.size();
    endEffectorPosition.reserve(pdat.numVertices());

    std::vector<unsigned int> edge_list;
    prevSize = edges.size();
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
            else
                endEffectorPosition.push_back(transform.translation());

            // edges handling
            unsigned int n_edge = pdat.getEdges(i, edge_list);
            for (unsigned int i2(0); i2 < n_edge; ++i2) {
                std::vector<Eigen::Vector3d> temp;
                temp.push_back(getVertex(pdat.getVertex(i)));
                temp.push_back(getVertex(pdat.getVertex(edge_list[i2])));
                edges.push_back(temp);
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
        cameraReset = !cameraReset;
    default:
        Win3D::keyboard(_key, _x, _y);
        break;
    }

    // Keyboard control for Controller

    glutPostRedisplay();
}
