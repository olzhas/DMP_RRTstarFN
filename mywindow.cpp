#include "mywindow.h"

#include <fstream>

//==============================================================================
MyWindow::MyWindow()
    : SimWindow()
{
    what = 0;
    why = 0;
    mZoom = 0.20;

}

//==============================================================================
MyWindow::~MyWindow()
{

}

//==============================================================================

void MyWindow::setMotion(og::PathGeometric *motion)
{
    motion_ = motion;
}

//==============================================================================
void MyWindow::timeStepping()
{
    // TODO implement draw visualization of the generated path
    // TODO draw the three, using markers?

    // simulate one step
    mWorld->step();
    dart::dynamics::Skeleton *staubli = mWorld->getSkeleton("TX90XLHB");


    if(why < motion_->getStateCount()){
        //std::cout<<motion_ ->getStateCount() << std::endl;
        double *jointSpace
                = (double*)motion_->getState(why)
                ->as<ob::RealVectorStateSpace::StateType>()->values;

        for (int i = 2; i < 8; ++i) {
            staubli->setPosition(i, jointSpace[i-2]);
        }
        staubli->computeForwardKinematics();
        why++;


    } else {
        mSimulating = false;
    }

}

//==============================================================================
void MyWindow::drawSkels()
{
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
        mWorld->getSkeleton(i)->draw(mRI);

    //drawTree();
}

//==============================================================================
void MyWindow::drawTree()
{
    for (int i = 0; i < endEffectorPosition.size(); ++i) {
        dart::gui::drawNode(endEffectorPosition.at(i));
    }
}

//==============================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
    switch (_key)
    {
    case ' ':  // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if (mSimulating)
        {
            mPlay = false;
            glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'p':  // playBack
        mPlay = !mPlay;
        if (mPlay)
        {
            mSimulating = false;
            glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[':  // step backward
        if (!mSimulating)
        {
            mPlayFrame--;
            if (mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']':  // step forwardward
        if (!mSimulating)
        {
            mPlayFrame++;
            if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    default:
        Win3D::keyboard(_key, _x, _y);
    }

    // Keyboard control for Controller

    glutPostRedisplay();
}

//==============================================================================
void MyWindow::initDrawTree()
{
    namespace bc = boost::chrono;
    bc::thread_clock::time_point start = bc::thread_clock::now();


    if (!ss_ || !ss_->haveSolutionPath()){
        std::cerr << "No solution =(" << std::endl;
        // return;
    }


    // Get the planner data to visualize the vertices and the edges
    ob::PlannerData pdat(ss_->getSpaceInformation());
    ss_->getPlannerData(pdat);

    // Print the vertices to file

    //result->node.resize(pdat.numVertices());
    dart::dynamics::Skeleton *staubli = mWorld->getSkeleton("TX90XLHB");

    endEffectorPosition.reserve(pdat.numVertices());

    for(unsigned int i(0); i<pdat.numVertices(); ++i)
    {
        std::vector<double> reals;
        if(pdat.getVertex(i)!=ob::PlannerData::NO_VERTEX)
        {
            ss_->getStateSpace()->copyToReals(reals, pdat.getVertex(i).getState());

            for(size_t j(0); j<reals.size(); ++j){
                staubli->setPosition(j+2, reals[j]);

                staubli->computeForwardKinematics();
                Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
                //Eigen::Vector3d
                //std::cout << transform.translation()<< std::endl;
                endEffectorPosition.push_back(transform.translation());
            }
        }
    }

    bc::thread_clock::time_point stop = bc::thread_clock::now();
    std::cout << "duration: "
              << bc::duration_cast<bc::milliseconds>(stop - start).count()
              << " ms\n";
}
