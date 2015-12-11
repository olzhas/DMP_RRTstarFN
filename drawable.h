#ifndef DRAWABLE_H
#define DRAWABLE_H

#include <vector>
#include <dart/dart.h>
#include <Eigen/Eigen>
#include <GL/freeglut.h>

class Drawable {

public:
    enum DrawableType { BOX,
        SPHERE };

    Drawable();
    void draw();
    void setType(DrawableType type) { type_ = type; }
    void setSize(double size) { size_ = size; }
    void setPoint(Eigen::Vector3d point) { point_ = point; }
    void setColor(Eigen::Vector3d color) { color_ = color; }

private:
    Eigen::Vector3d point_;
    Eigen::Vector3d color_;
    double size_;
    DrawableType type_;

    bool visible_;
};

class DrawableCollection {
private:
    std::vector<Drawable*> data_;
    std::string caption_;
    uint32_t id_;

    bool visible_ = true;

public:
    DrawableCollection() {;}
    DrawableCollection(Drawable* d) { add(d); }
    DrawableCollection(size_t size) { data_.reserve(size); }

    void add(Drawable* d) { data_.push_back(d); }
    void draw();

    void setCaption(std::string caption) { caption_ = caption; }
};
#endif // DRAWABLE_H
