#ifndef MODEL_H
#define MODEL_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <Eigen/Eigen>
#include <fstream>
#include <iostream>

namespace ob = ompl::base;

/**
 * \brief The Model class provides the collision detection model Circles, AABB,
 * OBB (not implemented yet)
 */
class Model {
 public:
  /** \brief the synonym type to describe the point in 2D space  */
  typedef Eigen::Vector2d Point;

  /**
 * \brief The Line class used to represent a line in 2D space
 */
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

  /**
   * \brief The Obstacle class
   */
  class Obstacle {
   private:
    std::string _name;

   public:
    Obstacle() { ; }
    virtual ~Obstacle() { ; }

    virtual bool detectCollision(const Obstacle* target) const { return true; }

    std::string getName() const { return _name; }

    void setName(const std::string& n) { _name = n; }
  };

  // TODO switch to smart pointer in C++11
  typedef std::shared_ptr<Obstacle> ObstaclePtr;

  /**
   * \brief The CircularObstacle class
   */
  class CircularObstacle : public Obstacle {
   private:
    Point pos;
    double radius_;
    double radiusSquare_;

   public:
    CircularObstacle() : radius_(0), radiusSquare_(0) { ; }

    virtual bool detectCollision(const Obstacle* target) const {
      const ObbObstacle* obb = static_cast<const ObbObstacle*>(target);

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

    // getters
    Point getPos() const { return pos; }
    double getRadius() const { return radius_; }
  };

  /**
   * \brief The ObbObstacle class, oriented bounding box, however at this moment
   * it is AABB
   */
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

    /**
     * \brief this method implements check for collision
     * \param target
     * \return
     */
    virtual bool detectCollision(const Obstacle* target) const;
    void calcSquareDiag() { squareDiag = width * width + height * height; }

    double getSquareDiag() const {
      const double factorOfSafety = 1.25;
      return factorOfSafety * squareDiag;
    }

    // getter
    Eigen::Vector2d getPos() const { return pos; }

    // setter
    void setPos(const Eigen::Vector2d& position) { pos = position; }

    /**
     * @brief move this methods used to move the object from place to place
     * (orientation is considered)
     * @param position in 2D space
     * @param y - yaw in radians
     */
    void move(const Eigen::Vector2d& position, const double y) {
      pos = position;
      yaw = y;
      update();
    }

    /**
     * \brief setWidth this method sets the width of the box
     * \param w - width of the object
     */
    void setWidth(const double& w) {
      width = w;
      calcSquareDiag();
      update();
    }

    /**
     * @brief setHeight this method set the height of the box
     * @param h - height of the object
     */
    void setHeight(const double& h) {
      height = h;
      calcSquareDiag();
      update();
    }

    /**
     * \brief update this method is used to calculate the internal data
     */
    void update() {
      // ugly initialization
      Eigen::MatrixXd temp(2, 4);

      // the order of the initialization of vertices
      // 3---0
      // |   |
      // 2---1

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

  class AABBObstacle : public Obstacle {
    virtual bool detectCollision(const Obstacle* object) const;

   private:
    Eigen::Vector2d pos;

    double width;
    double height;
  };

  /**
   * @brief The ObstacleCollection class
   */
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

  // ctor
  Model(const std::string& filename) : dynamicObstaclesState_(0) {
    mapFilename_ = filename;
    loadSimpleWorld();
  }

  void setDynamicObstaclesFile(std::string& filename);
  void loadSimpleWorld();
  bool isStateValid(const ob::State* state);

  void updateObstacles();
  void setSpaceInformation(const ob::SpaceInformationPtr& si) { si_ = si; }

  void loadObstacles(const std::string& fname, ObstacleCollection& collection);
  void loadDynamicObstacles(const std::string& filename,
                            ObstacleCollection& collection);

  /**
   * @brief createCircularObstacle
   * @param to_parse
   * @return
   */
  CircularObstacle* createCircularObstacle(const std::string& to_parse);

  /**
   * @brief createAABBObstacle
   * @param to_parse
   * @return
   */
  ObbObstacle* createAABBObstacle(const std::string& to_parse);

  /**
   * @brief createObbObstacle
   * @param to_parse
   * @return
   */
  ObbObstacle* createObbObstacle(const std::string& to_parse);

  /**
   * @brief loadTemporalData
   * @param fname
   */
  void loadTemporalData(const std::string& fname);

  /**
   * @brief updateEnvironment
   */
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
