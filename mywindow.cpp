#include "mywindow.h"

#include <fstream>

//==============================================================================
MyWindow::MyWindow()
    : SimWindow()
{
    what = 0;
    why = 0;


}

//==============================================================================
MyWindow::~MyWindow()
{

}

//==============================================================================
    /*
MyWindow::readResults()
{
    // read the nodes information

    std::ifstream fin("vertices.dat");
    size_t lines;
    std::string line;
    while(fin.eof()){
        std::getline(fin, line);
        lines++;
    }
    fin.clear();
    fin.seekg(0, std::ios::beg);
    visuNodes = malloc(lines*sizeof(double[2][6]));
    double visuState[6];
    for (int j = 0; j < lines; ++j) {
        for (int i = 0; i < 6; ++i) {
            fin >> visuState[i];
            staubli->setPosition(i+2, visuState[i]);
        }
        staubli->computeForwardKinematics();
        Eigen::Isometry3d transform = staubli->getBodyNode("toolflange_link")->getTransform();
        //Eigen::Vector3d
        //std::cout << transform.translation()<< std::endl;
        dart::gui::drawNode(transform.translation());
    }
    fin.close();

}
*/
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

        if(what < 10){
            for (int i = 2; i < 8; ++i) {
                staubli->setPosition(i, jointSpace[i-2]);
            }
            staubli->computeForwardKinematics();
        }else {why++; what =0;}
        what++;
    } else {
        mSimulating = false;
    }

    //std::cout << motion_->getStateCount() << std::endl;
}

//==============================================================================
void MyWindow::drawSkels()
{
    //glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
        mWorld->getSkeleton(i)->draw(mRI);

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
