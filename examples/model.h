#ifndef MODEL_H
#define MODEL_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <Eigen/Eigen>
#include <fstream>
#include <iostream>

namespace ob = ompl::base;

class Model {
 public:
  class Point {
    double x_;
    double y_;

   public:
    Point() { ; }
    Point(const Point& p) {
      x_ = p.x();
      y_ = p.y();
    }
    Point(const double& x, const double& y) {
      x_ = x;
      y_ = y;
    }

    Point& operator=(Point p) {
      x_ = p.x();
      y_ = p.y();
      return *this;
    }

    // getters
    double x() const { return x_; }
    double y() const { return y_; }

    Eigen::Vector2d toVector() const {
      Eigen::Vector2d out;
      out << x_, y_;
      return out;
    }
  };

  class Line {
    Point head_;
    Point tail_;

   public:
    Line(Point head, Point tail) : head_(head), tail_(tail) { ; }
    Line() : head_(Point(0.0, 0.0)), tail_(Point(0.0, 0.0)) { ; }

    // getters
    Point getHead() const { return head_; }
    Point getTail() const { return tail_; }

    double getLength() const {
      auto x = head_.x() - tail_.x();
      auto y = head_.y() - tail_.y();

      return sqrt(x * x + y * y);
    }

    Point middle() const {
      Point p((head_.x() + tail_.x()) / 2.0, (head_.y() + tail_.y()) / 2.0);
      return p;
    }
  };

  class Obstacle {
   private:
    std::string name;

   public:
    Obstacle() { ; }
    virtual ~Obstacle() { ; }

    virtual bool detectCollision(Obstacle* target) { return true; }

    std::string getName() const { return name; }

    void setName(const std::string& n) { name = n; }
  };

  class CircularObstacle : public Obstacle {
   private:
    Eigen::Vector2d pos;
    double radius;
    double radiusSquare;

   public:
    CircularObstacle() { ; }

    bool detectCollision(Obstacle* target) {
      ObbObstacle* obb = static_cast<ObbObstacle*>(target);

      Eigen::Vector2d diff = (obb->getPos() - pos);
      double squareDistance = diff.dot(diff);

      if (squareDistance < (radiusSquare + obb->getSquareDiag())) {
        return true;
      }
      return false;
    }

    void move(const Eigen::Vector2d& p, const double& r) {
      pos = p;
      radius = r;
      radiusSquare = r * r;
    }

    Eigen::Vector2d getPos() const { return pos; }

    double getRadius() const { return radius; }
  };

  class ObbObstacle : public Obstacle {
   public:
    size_t maxX, minX;
    size_t maxY, minY;

    Eigen::MatrixXd vertices;

    /** \brief constructor */
    ObbObstacle() : vertices(2, 4), yaw(0), width(0), height(0) {
      pos << 0, 0;
      update();
    }

    /** \brief constructor */
    ObbObstacle(const Eigen::Vector2d p, double y, double w, double h)
        : vertices(2, 4), yaw(y), width(w), height(h) {
      pos = p;
      calcSquareDiag();
      update();
    }

    /** \brief */
    bool detectCollision(Obstacle* target) {
      bool intersectX = false, intersectY = false;
      ObbObstacle* obb = static_cast<ObbObstacle*>(target);
      // Eigen::Vector2d diff = pos - obb->getPos();
      // double squareDistance = diff.dot(diff);

      ObbObstacle* left = pos[0] > obb->getPos()[0] ? obb : this;
      ObbObstacle* right = pos[0] > obb->getPos()[0] ? this : obb;

      double leftX = left->vertices(0, left->maxX) - left->getPos()[0];
      double rightX = right->getPos()[0] - right->vertices(0, right->minX);

      double x = right->getPos()[0] - left->getPos()[0];

      if (x < leftX + rightX) {
        intersectX = true;
      }

      ObbObstacle* down = pos[1] > obb->getPos()[1] ? obb : this;
      ObbObstacle* up = pos[1] > obb->getPos()[1] ? this : obb;

      double downY =
          down->vertices(1, static_cast<long>(down->maxY)) - down->getPos()[1];
      double upY =
          up->getPos()[1] - up->vertices(1, static_cast<long>(up->minY));

      double y = up->getPos()[1] - down->getPos()[1];

      if (y < downY + upY) {
        intersectY = true;
      }

      return intersectX & intersectY;
    }

    /** \brief */
    void calcSquareDiag() { squareDiag = width * width + height * height; }

    double getSquareDiag() const {
      const double factorOfSafety = 1.25;
      return factorOfSafety * squareDiag;
    }

    /** \brief */
    Eigen::Vector2d getPos() const { return pos; }

    /** \brief */
    void setPos(const Eigen::Vector2d& position) {
      pos = position;
      // update();
    }

    /** \brief */

    void move(const Eigen::Vector2d& position, const double y) {
      pos = position;
      yaw = y;
      update();
    }

    /** \brief */
    void setWidth(const double& w) {
      width = w;
      calcSquareDiag();
      update();
    }

    /** \brief */
    void setHeight(const double& h) {
      height = h;
      calcSquareDiag();
      update();
    }

    void update() {
      // ugly initialization
      Eigen::MatrixXd temp(2, 4);

      /*
3---0
|   |
2---1
*/
      temp << width / 2.0 * cos(yaw) - height / 2.0 * sin(yaw),
          width / 2.0 * cos(yaw) + height / 2.0 * sin(yaw),
          -width / 2.0 * cos(yaw) + height / 2.0 * sin(yaw),
          -width / 2.0 * cos(yaw) - height / 2.0 * sin(yaw),
          width / 2.0 * sin(yaw) + height / 2.0 * cos(yaw),
          width / 2.0 * sin(yaw) - height / 2.0 * cos(yaw),
          -width / 2.0 * sin(yaw) - height / 2.0 * cos(yaw),
          -width / 2.0 * sin(yaw) + height / 2.0 * cos(yaw);
      vertices = temp;
      // std::cout << vertices << std::endl;

      Eigen::MatrixXd offset(2, 4);
      offset << pos[0], pos[0], pos[0], pos[0], pos[1], pos[1], pos[1], pos[1];

      vertices += offset;

      Eigen::MatrixXd::Index max[2];
      Eigen::MatrixXd::Index min[2];

      for (long i = 0; i < 2; ++i) {
        vertices.row(i).maxCoeff(&max[i]);
        vertices.row(i).minCoeff(&min[i]);
      }

      maxX = static_cast<size_t>(max[0]);
      maxY = static_cast<size_t>(max[1]);

      minX = static_cast<size_t>(min[0]);
      minY = static_cast<size_t>(min[1]);
    }

   private:
    Eigen::Vector2d pos;
    double yaw;

    double width;
    double height;
    double squareDiag;
  };

  class ObstacleCollection {
   private:
    std::vector<Obstacle*> data_;

   public:
    bool detectCollision(Obstacle* ob) {
      for (auto& elem : data_) {
        if (elem->detectCollision(ob) == true) {
          return true;
        }
      }
      return false;
    }

    void add(Obstacle* a) { data_.push_back(a); }
  };

  Model(const std::string& filename) {
    mapFilename_ = filename;
    loadSimpleWorld();
  }

  void setDynamicObstaclesFile(std::string& filename) {
    std::ifstream fin(filename);
    std::ofstream fout("dynamic_before.gnu");

    assert(!fin.fail() && "cannot open file");
    assert(!fout.fail() && "cannot open file");

    double x, y, r;
    fin >> x >> y >> r;
    dynamicCircle_.resize(1);
    dynamicCircle_[0] = new CircularObstacle;
    dynamicCircle_[0]->move(Eigen::Vector2d(x, y), r);

    fout << "set object " << 8000 << " circle at " << x << "," << y << " size "
         << r << " fc rgb \"#FF4444\" front\n";

    fout.flush();
    fout.close();

    fout.open("dynamic_after.gnu");
    assert(!fout.fail() && "cannot open file");

    fin >> x >> y >> r;
    futurePosition_.push_back(x);
    futurePosition_.push_back(y);
    futurePosition_.push_back(r);

    fout << "set object " << 8000 << " circle at " << x << "," << y << " size "
         << r << " fc rgb \"#FF4444\" front\n";

    for (auto& obs : dynamicCircle_) {
      obstacles_.add(obs);
    }
  }
  void loadSimpleWorld() {
    loadObstacles(mapFilename_, obstacles_);

    simpleCar_.setWidth(80);
    simpleCar_.setHeight(50);
  }

  bool isStateValid(const ob::State* state) {
    // std::lock_guard<std::mutex> guard(mutex_);
    const ob::SE2StateSpace::StateType* s =
        state->as<ob::SE2StateSpace::StateType>();

    if (!si_->satisfiesBounds(s)) return false;

    double x = s->getX();
    double y = s->getY();
    double yaw = s->getYaw();

    simpleCar_.move(Eigen::Vector2d(x, y), yaw);
    return !obstacles_.detectCollision(&simpleCar_);
  }

  void updateObstacles() {
    dynamicCircle_[0]->move(
        Eigen::Vector2d(futurePosition_[0], futurePosition_[1]),
        futurePosition_[2]);
  }

  void setSpaceInformation(const ob::SpaceInformationPtr& si) { si_ = si; }

  void loadObstacles(const std::string& fname, ObstacleCollection& collection) {
    Obstacle* obs;
    std::string str;

    std::ifstream fin(fname);
    std::ofstream fout("obstacles.gnu");

    const std::size_t nColors = 6;
    const std::string colors[nColors] = {"#FF4136", "#39CCCC", "#3D9970",
                                         "#B10DC9", "#0074D9", "#F012BE"};

    if (fin) {
      size_t i = 100;

      while (!fin.eof()) {
        char type;
        fin >> type;
        std::getline(fin, str);

        switch (type) {
          case 'c':
          case 'C':
            obs = createCircularObstacle(str);
            fout << "set object " << ++i << " ";
            fout << "circle at "
                 << static_cast<CircularObstacle*>(obs)->getPos()[0] << ","
                 << static_cast<CircularObstacle*>(obs)->getPos()[1] << " size "
                 << static_cast<CircularObstacle*>(obs)->getRadius()
                 << " fc rgb \"#FF4444\" front" << std::endl;
            break;
          case 'r':
          case 'R':
            obs = createObbObstacle(str);
            fout << "set object " << ++i << " ";
            fout << "rect from "
                 << static_cast<ObbObstacle*>(obs)->vertices(0, 2) << ","
                 << static_cast<ObbObstacle*>(obs)->vertices(1, 2) << " to "
                 << static_cast<ObbObstacle*>(obs)->vertices(0, 0) << ","
                 << static_cast<ObbObstacle*>(obs)->vertices(1, 0)
                 << " fc rgb \"" << colors[i % nColors] << "\" front"
                 << std::endl;

            break;
          case 'b':
          case 'B':
            obs = createObbObstacle2(str);
            fout << "set object " << ++i << " ";
            fout << "rect from "
                 << static_cast<ObbObstacle*>(obs)->vertices(0, 2) << ","
                 << static_cast<ObbObstacle*>(obs)->vertices(1, 2) << " to "
                 << static_cast<ObbObstacle*>(obs)->vertices(0, 0) << ","
                 << static_cast<ObbObstacle*>(obs)->vertices(1, 0)
                 << " fc rgb \"" << colors[i % nColors] << "\" front"
                 << std::endl;

            break;

          default:
            obs = nullptr;
            break;
        }
        if (obs != nullptr) collection.add(obs);
      }
    } else {
      std::cerr << "could not open file " << fname << std::endl;
    }
  }

  CircularObstacle* createCircularObstacle(const std::string& to_parse) {
    double x, y, r;
    std::string name;
    std::istringstream iss(to_parse);

    iss >> x >> y >> r >> name;

    CircularObstacle* object = new CircularObstacle;
    object->move(Eigen::Vector2d(x, y), r);
    return object;
  }

  ObbObstacle* createObbObstacle(const std::string& to_parse) {
    double x, y;
    double width, height;

    std::istringstream iss(to_parse);

    iss >> x >> y >> width >> height;

    ObbObstacle* object = new ObbObstacle;

    object->setWidth(width);
    object->setHeight(height);
    object->move(Eigen::Vector2d(x, y), 0);
    return object;
  }

  ObbObstacle* createObbObstacle2(const std::string& to_parse) {
    double x, y;
    double width, height, yaw;

    std::istringstream iss(to_parse);

    iss >> x >> y >> width >> height >> yaw;

    ObbObstacle* object = new ObbObstacle;

    object->setWidth(width);
    object->setHeight(height);
    object->move(Eigen::Vector2d(x, y), yaw);
    return object;
  }

 private:
  ob::SpaceInformationPtr si_;

  ObstacleCollection obstacles_;
  std::vector<ObstacleCollection*> dynamicObstacles_;

  ObbObstacle simpleCar_;
  std::string mapFilename_;

  std::vector<CircularObstacle*> dynamicCircle_;
  std::vector<double> futurePosition_;
};

#endif  // MODEL_H
