#include "widget.h"

Widget::Widget(int *argcp, char **argv)
{
    init(argcp, argv);
}

void Widget::init(int *argcp, char **argv)
{
    window.setWorld(manipulator->getWorld());
    og::PathGeometric resultantMotion = manipulator->getResultantMotion();

    window.setMotion(&resultantMotion);
    window.ss_ = manipulator->ss_;
    window.initDrawTree();

    glutInit(argcp, argv);
    window.initWindow(800, 600, "Staubli TX90XL");
    glutMainLoop();
}

void Widget::setManipulator(ManipulatorPtr robot)
{
    manipulator = robot;
}
