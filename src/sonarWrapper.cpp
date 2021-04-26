/*
 * sonarWrapper.cpp
 * Copyright (C) 2021 Nick van der Merwe <nick.vandermerwe@griffithuni.edu.au
 *
 * The purpose of this is to wrap the sonars topic and return
 * what it says
 */
#include "ros/ros.h"
#include "assignment1_setup/Sonars.h"
#include "assignment1/getSonarReadings.h"
#include <inttypes.h>

class SonarReader{
    public:
        SonarReader() = default;

        void subCallback(const assignment1_setup::Sonars &msg){
            lastReading = msg;
        }

        bool getSonarReadings(
                assignment1::getSonarReadings::Request &req,
                assignment1::getSonarReadings::Response &res){
            res.readings = lastReading;
            ROS_INFO("Sending %u", (unsigned int)res.readings.distance1);
            return true;
        }
    private:
        assignment1_setup::Sonars lastReading;
};



int main(int argc, char **argv){
    ros::init(argc, argv, "sonar_wrapper_node");
    ros::NodeHandle nodeHandle;

    SonarReader sr{};
    ros::Subscriber sonarReader =
        nodeHandle.subscribe("sonars", 1000, 
                &SonarReader::subCallback, &sr);

    ros::ServiceServer getSonarReadings =
        nodeHandle.advertiseService("sonar_wrapper", 
                &SonarReader::getSonarReadings, &sr);

    ROS_INFO("Ready to manage requests");

    ros::spin();
    return EXIT_SUCCESS;
}
