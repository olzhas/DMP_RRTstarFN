#include "model.h"

Model::ObbObstacle* Model::createObbObstacle(const std::string& to_parse) {
  double x, y;
  double width, height;
  std::string name;

  std::istringstream iss(to_parse);

  iss >> x >> y >> width >> height >> name;

  ObbObstacle* object = new ObbObstacle;

  object->setWidth(width);
  object->setHeight(height);
  object->move(Eigen::Vector2d(x, y), 0);
  object->setName(name);
  return object;
}

void Model::loadTemporalData(const std::string& fname) {
  Obstacle* obs;
  std::string str;

  std::ifstream fin(fname);

  if (fin) {
    while (!fin.eof()) {
      int index;
      fin >> index;
      while (dynamicObstacles_.size() <= index) {
        dynamicObstacles_.push_back(nullptr);
      }
      int numCom;
      fin >> numCom;
      for (; numCom > 0; --numCom) {
        char type;
        fin >> type;
        std::getline(fin, str);
        switch (type) {
          case 'c':
          case 'C':
            obs = createCircularObstacle(str);
            break;
          case 'r':
          case 'R':
            obs = createObbObstacle(str);
            break;
          case 'b':
          case 'B':
            obs = createObbObstacle2(str);
            break;
          default:
            obs = nullptr;
            break;
        }

        if (obs != nullptr) dynamicObstacles_[index]->add(obs);
      }
    }
  } else {
    std::cerr << "could not open file " << fname << std::endl;
  }
}

void Model::updateEnvironment() {
  static std::size_t incrementalState = 0;
  ++incrementalState;
}

Model::ObbObstacle* Model::createObbObstacle2(const std::string& to_parse) {
  double x, y;
  double width, height, yaw;
  std::string name;

  std::istringstream iss(to_parse);

  iss >> x >> y >> width >> height >> yaw >> name;

  ObbObstacle* object = new ObbObstacle;

  object->setWidth(width);
  object->setHeight(height);
  object->move(Eigen::Vector2d(x, y), yaw);
  object->setName(name);
  return object;
}

Model::CircularObstacle* Model::createCircularObstacle(
    const std::string& to_parse) {
  double x, y, r;
  std::string name;
  std::istringstream iss(to_parse);

  iss >> x >> y >> r >> name;

  CircularObstacle* object = new CircularObstacle;
  object->move(Eigen::Vector2d(x, y), r);
  object->setName(name);
  return object;
}

void Model::loadObstacles(const std::string& fname,
                          ObstacleCollection& collection) {
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
          fout << "rect from " << static_cast<ObbObstacle*>(obs)->vertices(0, 2)
               << "," << static_cast<ObbObstacle*>(obs)->vertices(1, 2)
               << " to " << static_cast<ObbObstacle*>(obs)->vertices(0, 0)
               << "," << static_cast<ObbObstacle*>(obs)->vertices(1, 0)
               << " fc rgb \"" << colors[i % nColors] << "\" front"
               << std::endl;

          break;
        case 'b':
        case 'B':
          obs = createObbObstacle2(str);
          fout << "set object " << ++i << " ";
          fout << "rect from " << static_cast<ObbObstacle*>(obs)->vertices(0, 2)
               << "," << static_cast<ObbObstacle*>(obs)->vertices(1, 2)
               << " to " << static_cast<ObbObstacle*>(obs)->vertices(0, 0)
               << "," << static_cast<ObbObstacle*>(obs)->vertices(1, 0)
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

void Model::updateObstacles() {
  dynamicCircle_[0]->move(
      Eigen::Vector2d(futurePosition_[0], futurePosition_[1]),
      futurePosition_[2]);
}

bool Model::isStateValid(const ob::State* state) {
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

void Model::loadSimpleWorld() {
  loadObstacles(mapFilename_, obstacles_);

  simpleCar_.setWidth(80);
  simpleCar_.setHeight(50);
}

void Model::setDynamicObstaclesFile(std::string& filename) {
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

/** \brief */
bool Model::ObbObstacle::detectCollision(Obstacle* target) {
  bool intersectX = false, intersectY = false;
  ObbObstacle* obb = static_cast<ObbObstacle*>(target);
  // Eigen::Vector2d diff = pos - obb->getPos();
  // double squareDistance = diff.dot(diff);

  ObbObstacle* left = pos[0] > obb->getPos()[0] ? obb : this;
  ObbObstacle* right = pos[0] > obb->getPos()[0] ? this : obb;

  double leftX =
      left->vertices(0, static_cast<long>(left->maxX)) - left->getPos()[0];
  double rightX =
      right->getPos()[0] - right->vertices(0, static_cast<long>(right->minX));

  double x = right->getPos()[0] - left->getPos()[0];

  if (x < leftX + rightX) {
    intersectX = true;
  }

  ObbObstacle* down = pos[1] > obb->getPos()[1] ? obb : this;
  ObbObstacle* up = pos[1] > obb->getPos()[1] ? this : obb;

  double downY =
      down->vertices(1, static_cast<long>(down->maxY)) - down->getPos()[1];
  double upY = up->getPos()[1] - up->vertices(1, static_cast<long>(up->minY));

  double y = up->getPos()[1] - down->getPos()[1];

  if (y < downY + upY) {
    intersectY = true;
  }

  return intersectX & intersectY;
}
