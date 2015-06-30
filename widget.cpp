#include "widget.h"

Widget::Widget()
{
;
}
//==============================================================================
void Widget::init()
{
    window.setWorld(manipulator->getWorld());
    og::PathGeometric resultantMotion = manipulator->getResultantMotion();

    window.setMotion(&resultantMotion);
    window.ss_ = manipulator->ss_;
    window.initDrawTree();

}
//==============================================================================
void Widget::setManipulator(ManipulatorPtr robot)
{
    manipulator = robot;
}
//==============================================================================
void Widget::exec(int *argcp, char **argv)
{
    glutInit(argcp, argv);
    window.initWindow(800, 600, "Staubli TX90XL");
    glutMainLoop();
}
