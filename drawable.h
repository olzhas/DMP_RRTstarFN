#ifndef DRAWABLE_H
#define DRAWABLE_H

#include <vector>
#include <dart/dart-core.h>
#include <ompl/base/State.h>

#ifdef WIN32
#include <cstdlib> // To disable glut::exit() function
#include <GL/glut.h>
#elif defined(__linux__)
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <Glut/glut.h>
#else
#error "Load OpenGL Error: What's your operating system?"
#endif

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
        , color_(0.8, 0.8, 0.8, 1.0)
        , type_(BOX)
        , visible_(DrawableVisibility::VISIBLE)
    {
        s_ = nullptr;
    }

    Drawable(Eigen::Vector3d point, Eigen::Vector4d color, double size,
             DrawableType type, DrawableVisibility visible = DrawableVisibility::VISIBLE)
        : size_(size)
        , type_(type)
        , visible_(visible)
    {
        point_ = point;
        color_ = color;
        s_ = nullptr;
    }

    Drawable(Eigen::Vector3d point, Eigen::Vector3d color, double size,
             DrawableType type, DrawableVisibility visible = DrawableVisibility::VISIBLE)
        : size_(size)
        , type_(type)
        , visible_(visible)
    {
        point_ = point;
        color_ << color[0], color[1], color[2], 1.0;
        s_ = nullptr;
    }

    virtual ~Drawable()
    {
        ;
    }

    virtual void draw();

    /* setters */
    void setType(DrawableType type) { type_ = type; }
    void setSize(double size) { size_ = size; }
    void setPoint(Eigen::Vector3d point) { point_ = point; }
    void setColor(Eigen::Vector3d color)
    {
        // drawable is opaque by default
        color_ << color[0], color[1], color[2], 1;
    }
    void setColor(Eigen::Vector4d color) { color_ = color; }
    void setState(ompl::base::State* s) { s_ = s; }

    /* getters */
    DrawableType getType() { return type_; }
    double getSize() { return size_; }
    Eigen::Vector3d getPoint() { return point_; }
    Eigen::Vector4d getColor() { return color_; }
    ompl::base::State* getState() { return s_; }

protected:
    double size_;
    Eigen::Vector3d point_;
    Eigen::Vector4d color_;
    DrawableType type_;
    DrawableVisibility visible_;
    ompl::base::State* s_;
};
//==============================================================================
class DrawableLiveTime : public Drawable {
public:
    DrawableLiveTime()
        : stepSize_(0.001)
        , sizeLimit_(0.001)
        , liveTime_(size_)
    {
        ;
    }
    virtual void draw() override;

    double calcSize()
    {
        if (liveTime_ > sizeLimit_)
            liveTime_ -= stepSize_;
        return liveTime_;
    }

    void setLiveTime(double l) { liveTime_ = l; }

private:
    double liveTime_;
    double stepSize_;
    double sizeLimit_;


};
//==============================================================================
class DrawableEdge : public Drawable {

    Eigen::Vector3d start_;
    Eigen::Vector3d end_;

public:
    DrawableEdge()
    {
        ;
    }
    DrawableEdge(Eigen::Vector3d start, Eigen::Vector3d end)
        : start_(start)
        , end_(end)
    {
        ;
    }
    void draw();

    // setters
    void setStart(Eigen::Vector3d start) { start_ = start; }
    void setEnd(Eigen::Vector3d end) { end_ = end; }

    // getters
    Eigen::Vector3d getStart() { return start_; }
    Eigen::Vector3d getEnd() { return end_; }
};
//==============================================================================
class DrawableCollection {
private:
    std::vector<Drawable*> data_;
    Drawable::DrawableVisibility visible_;
    std::string caption_;
    uint32_t id_;

    std::mutex dcMutex_;

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
    Drawable* getElement(size_t n)
    {
        if (n < data_.size())
            return data_[n];
        return nullptr;
    }

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

typedef std::shared_ptr<DrawableCollection> DrawableCollectionPtr;

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
