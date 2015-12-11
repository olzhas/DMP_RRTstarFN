#include "drawable.h"

Drawable::Drawable():
    size_(0.01),
    point_(0,0,0),
    color_(0,0,0)
{

}

void Drawable::draw()
{
    GLUquadricObj* c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);

    glColor3d(1.0, 0.2, 0.2);

    glPushMatrix();
    glTranslatef(point_[0], point_[1], point_[2]);
    glutSolidCube(size_);
    glPopMatrix();

    gluDeleteQuadric(c);
}
