#include "guimisc.h"

namespace dart
{
namespace gui
{
//==============================================================================
void drawLine3D(const Eigen::Vector3d start, const Eigen::Vector3d end)
{
    double _thickness = 0.0005;

    Eigen::Vector3d normDir = end - start;
    double _length = normDir.norm();
    normDir.normalize();

    // draw the arrow body as a cylinder
    GLUquadricObj *c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);
    //    GLfloat color[4]={0.8, 0.1, 0, 1};
    glColor3d(239.0/255.0,113.0/255.0,38.0/255.0);


    glPushMatrix();
    glTranslatef(start[0], start[1], start[2]);
    glRotated(acos(normDir[2])*180/M_PI, -normDir[1], normDir[0], 0);
    gluCylinder(c, _thickness, _thickness, _length, 4, 4);

    glPopMatrix();

    gluDeleteQuadric(c);

}

//==============================================================================
void drawSphere(const Eigen::Vector3d center, double radius)
{
    GLUquadricObj *c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);

    glPushMatrix();
    glTranslatef(center[0], center[1], center[2]);
    gluSphere(c, radius, 64, 64);
    glPopMatrix();

    gluDeleteQuadric(c);
}

//==============================================================================
void drawNode(const Eigen::Vector3d center)
{
    GLUquadricObj *c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);
    glPushMatrix();
    glColor3d(165.0/255.0, 225.0/255.0, 43.0/255.0);
    glTranslatef(center[0], center[1], center[2]);
    glutSolidCube(0.5);
    glPopMatrix();
    gluDeleteQuadric(c);
}

}
}
