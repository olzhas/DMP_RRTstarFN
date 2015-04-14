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

  // simulate one step
  mWorld->step();

}

//==============================================================================
void MyWindow::drawSkels()
{
  glEnable(GL_LIGHTING);
  //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

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
