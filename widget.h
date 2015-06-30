#ifndef WIDGET_H_
#define WIDGET_H_

#include "mywindow.h"
#include "manipulator.h"

class Widget
{
public:
    Widget(int *argcp, char **argv);
    void init(int *argcp, char **argv);
    void setManipulator(ManipulatorPtr robot);

private:
    MyWindow window;
    ManipulatorPtr manipulator;

};

#endif // WIDGET_H_
