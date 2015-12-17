#include "obstaclemanager.h"

void ObstacleManager::loadAll()
{
    YAML::Node config = YAML::LoadFile(configFileName_);
    std::vector<std::string> obsNames;

    if (config["active-obstacles"]) {
        if (config["active-obstacles"].size() > 0) {
            for (size_t i = 0; i < config["active-obstacles"].size(); ++i) {
                obsNames.push_back(config["active-obstacles"][i].as<std::string>());
            }
        }
    }
    else {
        dtwarn << "Possible error";
        dtwarn << "No active obstacles";
    }


    for (size_t i = 0; i < obsNames.size(); ++i) {
        dd::SkeletonPtr obsSkel(du::SkelParser::readSkeleton(OBSTACLE_PATH + obsNames[i]));
        if(obsSkel != nullptr){
        Obstacle* obstacle = new Obstacle(obsSkel, obsNames[i]);
        obstacles_.push_back(obstacle);
        }
        else {
            dterr << "Cannot load obstacle skel file\n";
        }
    }

}

void ObstacleManager::spawn(const std::string& name)
{
    Obstacle *obstacle;
    for(size_t i = 0; i < obstacles_.size(); ++i){
        if(obstacles_[i]->getName() == name){
            obstacle = obstacles_[i];
            break;
        }
    }
    world_->addSkeleton(obstacle->getSkeleton());
}
