#include "drawable.h"

Drawable::Drawable()
    : size_(0.01)
    , point_(0, 0, 0)
    , color_(0.8, 0.8, 0.8)
    , type_(BOX)
    , visible_(true)
{
}

void Drawable::draw()
{
    GLUquadricObj* c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);

    glColor3d(color_[0], color_[1], color_[2]);

    glPushMatrix();
    glTranslatef(point_[0], point_[1], point_[2]);
    switch (type_) {
    case BOX:
        glutSolidCube(size_);
        break;
    case SPHERE:
        gluSphere(c, size_, 6, 6);
        break;
    default:
        dtwarn << "DrawableType is no specified\n";
    }

    glPopMatrix();

    gluDeleteQuadric(c);
}

void DrawableCollection::draw()
{
    for (auto it = data_.begin(); it != data_.end(); ++it) {
        Drawable* d = *it;
        d->draw();
    }
}
