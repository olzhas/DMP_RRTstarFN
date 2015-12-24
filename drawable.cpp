#include "drawable.h"

//==============================================================================
void Drawable::draw()
{
    if(visible_ == Drawable::DrawableVisibility::HIDDEN)
        return;

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
        glutSolidSphere(size_, 6, 6);
        break;
    default:
        dtwarn << "DrawableType is no specified\n";
    }

    glPopMatrix();

    gluDeleteQuadric(c);
}
//==============================================================================
void DrawableLiveTime::draw()
{
    if(visible_ == Drawable::DrawableVisibility::HIDDEN)
        return;

    GLUquadricObj* c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);

    glColor4d(color_[0], color_[1], color_[2], 0.5);

    glPushMatrix();
    glTranslatef(point_[0], point_[1], point_[2]);
    size_ = calcSize();
    switch (type_) {
    case BOX:
        glutSolidCube(size_);
        break;
    case SPHERE:
        glutSolidSphere(size_, 6, 6);
        break;
    default:
        dtwarn << "DrawableType is no specified\n";
    }

    glPopMatrix();

    gluDeleteQuadric(c);
}
//==============================================================================
void DrawableCollection::draw()
{
    if (getVisibility() == Drawable::DrawableVisibility::HIDDEN)
        return;
    size_t end = data_.size();
    for (size_t i = 0; i < end; ++i) {
        Drawable* d = data_[i];
        d->draw();
    }
}
//==============================================================================
void drawLine3D(const Eigen::Vector3d start, const Eigen::Vector3d end)
{
    double _thickness = 0.001;

    Eigen::Vector3d normDir = end - start;
    double _length = normDir.norm();
    normDir.normalize();

    // draw the arrow body as a cylinder
    GLUquadricObj* c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);
    //    GLfloat color[4]={0.8, 0.1, 0, 1};

    glColor4d(25.0 / 255.0, 102.0 / 255.0, 0.0 / 255.0, 0.4);

    glPushMatrix();
    glTranslatef(start[0], start[1], start[2]);
    glRotated(acos(normDir[2]) * 180 / M_PI, -normDir[1], normDir[0], 0);
    gluCylinder(c, _thickness, _thickness, _length, 4, 4);

    glPopMatrix();

    gluDeleteQuadric(c);
}