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
  /** \brief */
  typedef Eigen::Vector2d Point;

  /** \brief */

  class Line {
    Point _head;
    Point _tail;

   public:
    Line(Point head, Point tail) : _head(head), _tail(tail) { ; }
    Line() : _head(Point(0.0, 0.0)), _tail(Point(0.0, 0.0)) { ; }

    // getters
    Point getHead() const { return _head; }
    Point getTail() const { return _tail; }

    double getLength() const {
      auto x = _head.x() - _tail.x();
      auto y = _head.y() - _tail.y();

      return sqrt(x * x + y * y);
    }

    Point middle() const {
      Point p((_head.x() + _tail.x()) / 2.0, (_head.y() + _tail.y()) / 2.0);
      return p;
    }
  };

  class Obstacle {
   private:
    std::string _name;

   public:
    Obstacle() { ; }
    virtual ~Obstacle() { ; }

    virtual bool detectCollision(Obstacle* target) { return true; }

    std::string getName() const { return _name; }

    void setName(const std::string& n) { _name = n; }
  };

  // TODO switch to smart pointer in C++11
  typedef std::shared_ptr<Obstacle> ObstaclePtr;

  class CircularObstacle : public Obstacle {
   private:
    Point pos;
    double radius_;
    double radiusSquare_;

   public:
    CircularObstacle() : radius_(0), radiusSquare_(0) { ; }

    bool detectCollision(Obstacle* target) {
      ObbObstacle* obb = static_cast<ObbObstacle*>(target);

      Eigen::Vector2d diff = (obb->getPos() - pos);
      double squareDistance = diff.dot(diff);

      if (squareDistance < (radiusSquare_ + obb->getSquareDiag())) {
        return true;
      }
      return false;
    }

    void move(const Point& p, const double& r) {
      pos = p;
      radius_ = r;
      radiusSquare_ = r * r;
    }

    Point getPos() const { return pos; }

    double getRadius() const { return radius_; }
  };

  /** \brief */
  class ObbObstacle : public Obstacle {
   public:
    Eigen::MatrixXd::Index maxX, minX;
    Eigen::MatrixXd::Index maxY, minY;

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
    bool detectCollision(Obstacle* target);
    /** \brief */
    void calcSquareDiag() { squareDiag = width * width + height * height; }

    double getSquareDiag() const {
      const double factorOfSafety = 1.25;
      return factorOfSafety * squareDiag;
    }

    /** \brief */
    Eigen::Vector2d getPos() const { return pos; }

    /** \brief */
    void setPos(const Eigen::Vector2d& position) { pos = position; }

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
#ifdef DEBUG
// TODO replace std::cout with OMPL_WARN()
// OMPL_WARN()
// std::cout << vertices << std::endl;
#endif

      Eigen::MatrixXd offset(2, 4);
      offset << pos[0], pos[0], pos[0], pos[0], pos[1], pos[1], pos[1], pos[1];

      vertices += offset;

      Eigen::MatrixXd::Index max[2];
      Eigen::MatrixXd::Index min[2];

      for (long i = 0; i < 2; ++i) {
        vertices.row(i).maxCoeff(&max[i]);
        vertices.row(i).minCoeff(&min[i]);
      }

      maxX = max[0];
      maxY = max[1];

      minX = min[0];
      minY = min[1];
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

  void setDynamicObstaclesFile(std::string& filename);
  void loadSimpleWorld();
  bool isStateValid(const ob::State* state);

  void updateObstacles();
  void setSpaceInformation(const ob::SpaceInformationPtr& si) { si_ = si; }

  void loadObstacles(const std::string& fname, ObstacleCollection& collection);

  /** \brief */
  CircularObstacle* createCircularObstacle(const std::string& to_parse);

  /** \brief */
  ObbObstacle* createObbObstacle(const std::string& to_parse);

  /** \brief */
  ObbObstacle* createObbObstacle2(const std::string& to_parse);

  /** \brief */
  void loadTemporalData(const std::string& fname);

  /** \brief */
  void updateEnvironment();

 private:
  ob::SpaceInformationPtr si_;

  ObstacleCollection obstacles_;
  std::vector<ObstacleCollection*> dynamicObstacles_;
  std::size_t dynamicObstaclesState_;

  ObbObstacle simpleCar_;
  std::string mapFilename_;

  std::vector<CircularObstacle*> dynamicCircle_;
  std::vector<double> futurePosition_;
};

#endif  // MODEL_H
