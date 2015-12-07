
#ifndef FRONTEND_H_
#define FRONTEND_H_

#include "mywindow.h"
#include "manipulator.h"

class Frontend {
public:
    Frontend();
    void init();
    void exec(int* argcp, char** argv);
    void setManipulator(ManipulatorPtr& robot);

    void loop();

private:
    MyWindowPtr pWindow;
    ManipulatorPtr pManipulator;
    dart::simulation::WorldPtr pRenderWorld;
};

typedef std::shared_ptr<Frontend> FrontendPtr;

#endif // FRONTEND_H_
