#include "obstaclemanager.h"

void ObstacleManager::loadAll()
{
    YAML::Node config = YAML::LoadFile(configFileName_);
    std::vector<std::string> obsNames;

    if(config["active-obstacles"]){
        if(config["active-obstacles"].size() > 0) {
            for(int i=0; i<config["active-obstacles"].size(); ++i){
                obsNames.push_back(config["active-obstacles"][i].as<std::string>());
            }
        }
    } else {
        dtwarn << "Possible error";
        dtwarn << "No active obstacles";
    }

    for(int i = 0; i < obsNames.size(); ++i){
        dd::SkeletonPtr obsSkel(du::SkelParser::readSkeleton(OBSTACLE_PATH + obsNames[i]));
        Obstacle* obstacle = new Obstacle(obsSkel, obsNames[i]);
    }
}
