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
    , prevSize(0)
{
    mZoom = 0.3f;
}
//==============================================================================

void MyWindow::initGhostManipulators()
{
    dd::SkeletonPtr staubliStartState(du::SdfParser::readSkeleton(SAFESPACE_DATA "/safespace/model.sdf"));
    staubliStartState->setName("TX90XLHB-Start");
    for (size_t i = 2; i < 8; ++i) {
        staubliStartState->setPosition(i, cfg->startState[i - 2]);
    }

    setSkeletonAlpha(staubliStartState, 0.2);
    setSkeletonCollidable(staubliStartState, false);
    mWorld->addSkeleton(staubliStartState);

    //==========================================================================

    dd::SkeletonPtr staubliFinalState = staubliStartState->clone();
    staubliFinalState->setName("TX90XLHB-Final");
    for (size_t i = 2; i < 8; ++i) {
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
#define ROBOT_NAME "TX90XLHB"
    // solutionPaths[0] is the active solution
    SolutionPath* sp = solutionPaths.back();
    std::vector<double>& st = sp->getNextState();
    if (st.size() == 0) {
        mSimulating = false;
        dtwarn << "No next state\n";
        return;
    }

    dd::SkeletonPtr robot(mWorld->getSkeleton(ROBOT_NAME));

    for (size_t i(0); i < st.size(); ++i) {
        robot->setPosition(i + 2, st[i]);
    }
    robot->computeForwardKinematics(true, false, false);
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

        dart::gui::drawStringOnScreen(0.02, 0.05, "camera fixed");
    }

    // just a text that indicates the progress
    static int progress = 0;
    if(!cfg->planningDone){
        std::string s = "planning";
        progress++;
        for(int i=0;i<progress/4;++i){
            s += ".";
        }
        if (progress > 12){
            progress = 0;
        }
        dart::gui::drawStringOnScreen(0.02, 0.075, s);
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
    //timer1.print();
    //timer1.stop();

    size_t end = drawables.size();
    for (size_t i = 0; i < end; ++i) {
        DrawableCollection* dc = drawables[i];
        dc->draw();
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

#define TREE_NAME "tree"

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
        for (auto it = drawables.begin(); it != drawables.end(); ++it) {
            DrawableCollection* dc = *it;
            if (dc->getCaption() == TREE_NAME) {
                dc->toggleVisibility();
                break;
            }
        }
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
