#include <thread>
#include <chrono>
#include <fstream>
#include <dart/dart.h>
#include "mywindow.h"

namespace dd = dart::dynamics;
namespace ds = dart::simulation;
namespace du = dart::utils;

#define SAFESPACE_DATA "/home/olzhas/devel/staubli_dart/data"

void movethread(dd::SkeletonPtr car)
{
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::ifstream fin("/home/olzhas/devel/build-staubli_dart-Desktop-Default/dubins-results-interp239.txt");

    for (int j = 0; !fin.eof(); ++j) {
        double x, y, yaw;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        fin >> x >> y >> yaw;
        Eigen::Isometry3d transform;
        transform.setIdentity();

        Eigen::Vector3d translation;

        translation[0] = x/250.0-4;
        translation[1] = y/250.0-4;
        translation[2] = 0;

        transform.translate(translation);

        Eigen::Matrix3d m;
        m.setIdentity();
        m = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        transform.rotate(m);

        dart::dynamics::FreeJoint::setTransform(car.get(), transform);
    }
}

int main(int argc, char** argv)
{
    ds::WorldPtr myWorld(du::SkelParser::readWorld(
        dart::common::Uri::createFromString(
            SAFESPACE_DATA "/ground_plane/ground.skel")));

    dd::SkeletonPtr car = du::SdfParser::readSkeleton(SAFESPACE_DATA "/car_a8/model.sdf");
    myWorld->addSkeleton(car);

    std::thread update(movethread, car);

    dart::gui::SimWindow window;
    window.setWorld(myWorld);
    glutInit(&argc, argv);
    window.initWindow(1280, 800, "0");
    window.refreshTimer(10);
    glutMainLoop();
}
