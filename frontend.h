#ifndef FRONTEND_H_
#define FRONTEND_H_

#include "mywindow.h"
#include "manipulator.h"

class Frontend {
public:
    Frontend();
    void init();
    void exec(int* argcp, char** argv);
    void setManipulator(ManipulatorPtr robot);

private:
    MyWindow window;
    ManipulatorPtr manipulator;
    dart::simulation::WorldPtr renderWorld;
};

#endif // WIDGET_H_
