/*
 * sonarWrapper.cpp
 * Copyright (C) 2021 Nick van der Merwe <nick.vandermerwe@griffithuni.edu.au
 *
 * The purpose of this is to wrap the sonars topic and return
 * what it says
 */
#include "ros/ros.h"
#include "assignment1_setup/Sonars.h"
#include <list>
#include <vector>
#include "assignment1/getSonarReadings.h"

class SonarReader{
    /**
     * This manages our sonar subscriber by appending
     * what it reads to a list then offers a service
     * which returns however many of those readings
     */
    public:
        void subCallback(const assignment1_setup::Sonars &msg){
            sonarInfo.push_back(msg);
        }

        bool getSonarReadings(
                assignment1::getSonarReadings::Request &req,
                assignment1::getSonarReadings::Response &res){
            res.readings.reserve(req.readingsToReturn);
            for(int i = 0; i < req.readingsToReturn; i++){
                res.readings.push_back(this->sonarInfo.front());
                this->sonarInfo.pop_front();
                if(sonarInfo.empty()){
                    // it'll be able to read the size of the std::vector
                    break;
                }
            }
            return true;
        }
    private:
        std::list<assignment1_setup::Sonars> sonarInfo;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "sonar_wrapper");
    ros::NodeHandle nodeHandle;

    SonarReader sr{};

    ros::Subscriber sonarReader =
        nodeHandle.subscribe("sonars", 1000, 
                &SonarReader::subCallback, &sr);

    ros::ServiceServer getSonarReadings =
        nodeHandle.advertiseService("getSonarReadings", 
                &SonarReader::getSonarReadings, &sr);

    ROS_INFO("Ready to manage requests");

    ros::spin();
    return EXIT_SUCCESS;
}
