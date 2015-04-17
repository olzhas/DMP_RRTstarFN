#include "mywindow.h"

//==============================================================================
MyWindow::MyWindow()
    : SimWindow()
{
}

//==============================================================================
MyWindow::~MyWindow()
{

}

//==============================================================================
void MyWindow::timeStepping()
{
    // TODO implement draw visualization of the generated path
    // TODO draw the three, using markers?

    // simulate one step
    mWorld->step();
    dart::dynamics::Skeleton *staubli = mWorld->getSkeleton("TX90XLHB");
    /*
    if(what < 20){
        for (int i = 2; i < 8; ++i) {
            staubli->setPosition(i, 0.5);
        }
        staubli->computeForwardKinematics();
    }else{
        //boost::this_thread::sleep_for(boost::chrono::milliseconds(500));

        for (int i = 2; i < 8; ++i) {
            staubli->setPosition(i, -1);
        }
        staubli->computeForwardKinematics();
        if(what > 40)
            what = 0;
    }
    what++;
    */
}

//==============================================================================
void MyWindow::drawSkels()
{
    glEnable(GL_LIGHTING);
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
