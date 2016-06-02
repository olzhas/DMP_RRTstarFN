#ifndef SOLUTIONPATH_H
#define SOLUTIONPATH_H

// general includes
#include <map>
#include <vector>

// dart includes
#include <dart/dart.h>
#include <Eigen/Eigen>

// OMPL includes
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// local includes
#include "drawable.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace dd = dart::dynamics;

class SolutionPath {
 private:
  DrawableCollection dc_;
  std::vector<ob::State*> states_;
  std::vector<Eigen::Isometry3d> poses_;

  ob::SpaceInformationPtr si_;
  std::string caption_;
  Eigen::Vector4d color_;

  size_t step = 0;

 public:
  SolutionPath() : caption_(""), color_({0.2, 0.4, 0.8, 1.0}) { ; }
  SolutionPath(std::string caption)
      : caption_(caption), color_({0.2, 0.4, 0.8, 1.0}) {
    ;
  }
  SolutionPath(std::string caption, std::string colorName)
      : caption_(caption), color_({0.2, 0.4, 0.8, 1.0}) {
    std::map<std::string, Eigen::Vector4d> colorMap;
    colorMap["r"] = {1.0, 0.0, 0.0, 0.1};
    colorMap["g"] = {0.0, 1.0, 0.0, 0.1};
    colorMap["b"] = {0.0, 0.0, 1.0, 0.1};

    auto it = colorMap.find(colorName);
    if (it != colorMap.end()) {
      color_ = *it;
    }
  }

  // setters

  void set(const og::PathGeometric& p, const ob::SpaceInformationPtr& si,
           const dd::SkeletonPtr& robot,
           // following argument does not change anything
           const Eigen::Vector4d& color = {0.2, 0.4, 0.8, 1.0},
           const double& size = 0.0175);

  void setCaption(const std::string& caption) { caption_ = caption; }

  // getters
  DrawableCollection& getDrawables() { return dc_; }
  std::string& getCaption() { return caption_; }

  std::vector<double>& getNextState();
};

#endif  // SOLUTIONPATH_H
