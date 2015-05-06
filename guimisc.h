#ifndef GUIMISC_H
#define GUIMISC_H

#include <dart/math/math.h>

#if WIN32
#include <cstdlib> // To disable glut::exit() function
#include <GL/glut.h>
#elif defined(__linux__)
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <Glut/glut.h>
#else
#error "Load OpenGL Error: What's your operating system?"
#endif

namespace dart
{
namespace gui
{
void drawLine3D(const Eigen::Vector3d start, const Eigen::Vector3d end);
void drawSphere(const Eigen::Vector3d center, double radius);
void drawNode(const Eigen::Vector3d center);
}
}


#endif // GUIMISC_H

