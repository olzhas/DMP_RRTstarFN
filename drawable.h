#ifndef DRAWABLE_H
#define DRAWABLE_H

#include <vector>
#include <dart/dart-core.h>

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

//TODO think how to include edge drawing routine

typedef Eigen::Vector4d Color;

//==============================================================================
class Drawable {

public:
    enum DrawableType { BOX,
        SPHERE };

    enum DrawableVisibility : bool { VISIBLE = 1,
        HIDDEN = 0 };

    Drawable()
        : size_(0.01)
        , point_(0, 0, 0)
        , color_(0.8, 0.8, 0.8)
        , type_(BOX)
        , visible_(DrawableVisibility::VISIBLE)
    {
    }

    Drawable(Eigen::Vector3d point, Eigen::Vector3d color, double size, DrawableType type, DrawableVisibility visible)
        : size_(size)
        , type_(type)
        , visible_(visible)
    {
        point_ = point;
        color_ = color;
    }

    virtual void draw();

    /* setters */
    void setType(DrawableType type) { type_ = type; }
    void setSize(double size) { size_ = size; }
    void setPoint(Eigen::Vector3d point) { point_ = point; }
    void setColor(Eigen::Vector3d color) { color_ = color; }

    /* getters */
    DrawableType getType() { return type_; }
    double getSize() { return size_; }
    Eigen::Vector3d getPoint() { return point_; }
    Eigen::Vector3d getColor() { return color_; }

protected:
    Eigen::Vector3d point_;
    Eigen::Vector3d color_;
    double size_;
    DrawableType type_;
    DrawableVisibility visible_;
};
//==============================================================================
class DrawableLiveTime : public Drawable {
    double liveTime_;
    double stepSize_;
    double sizeLimit_;

public:
    void draw() override;

    double calcSize()
    {
        if (liveTime_ > sizeLimit_)
            liveTime_ -= stepSize_;
        return liveTime_;
    }
};
//==============================================================================
class DrawableCollection {
private:
    std::vector<Drawable*> data_;
    std::string caption_;
    uint32_t id_;

    Drawable::DrawableVisibility visible_;

public:
    DrawableCollection()
        : visible_(Drawable::DrawableVisibility::VISIBLE)
    {
        caption_ = ""; // TODO use dart name server
    }
    DrawableCollection(Drawable* d)
        : visible_(Drawable::DrawableVisibility::VISIBLE)
    {
        add(d);
    }
    DrawableCollection(size_t size)
        : visible_(Drawable::DrawableVisibility::VISIBLE)
    {
        data_.reserve(size);
    }
    DrawableCollection(const std::string& caption)
        : visible_(Drawable::DrawableVisibility::VISIBLE)
        , caption_(caption)
    {
    }

    void add(Drawable* d) { data_.push_back(d); }
    void draw();

    /* setters */
    void setCaption(std::string caption) { caption_ = caption; }
    void setVisibility(Drawable::DrawableVisibility visible) { visible_ = visible; }
    void toggleVisibility()
    {
        switch (visible_) {
        case Drawable::DrawableVisibility::VISIBLE:
            visible_ = Drawable::DrawableVisibility::HIDDEN;
            break;
        case Drawable::DrawableVisibility::HIDDEN:
            visible_ = Drawable::DrawableVisibility::VISIBLE;
            break;
        }
    }

    /* getters */
    std::string getCaption() { return caption_; }
    Drawable::DrawableVisibility getVisibility() { return visible_; }

    size_t size() { return data_.size(); }
};

namespace dart {
namespace gui {
    class SimpleRGB {
    public:
        double r;
        double g;
        double b;

        SimpleRGB(double red, double green, double blue)
        {
            r = red;
            g = green;
            b = blue;
        }
    };
}
}
#endif // DRAWABLE_H
