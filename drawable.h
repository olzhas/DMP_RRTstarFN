#ifndef DRAWABLE_H
#define DRAWABLE_H

#include <Eigen/Eigen>
#include <vector>

#include <GL/freeglut.h>

class Drawable
{

private:
    Eigen::Vector3d point_;
    Eigen::Vector3d color_;
    double size_;


public:
    Drawable();

    void draw();
};

class DrawableCollection{
private:
    std::vector<Drawable*> data_;
public:
    DrawableCollection();
};
#endif // DRAWABLE_H
