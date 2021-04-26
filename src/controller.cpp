#include "ros/ros.h"
#include "assignment1_setup/Sonars.h"
#include "assignment1/getSonarReadings.h"
#include "assignment1/pidAlgorithm.h"
#include "assignment1/kalmanEstimateVariance.h"
#include "assignment1/kalmanFilter.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

// Maximum is 1.6 radians / second -> half a radian
// One degree = 180/M_PI
constexpr double TURNRATE = 180/M_PI; 

void defineTurn(geometry_msgs::Twist & defineTurnOf, 
                    assignment1::getSonarReadings sonarReadingSrv){
    // We need to rotate
    if(sonarReadingSrv.response.readings.distance0 == UINT16_MAX || 
        sonarReadingSrv.response.readings.distance2 == UINT16_MAX){
        // Unable to see where the robot is so just turn right
        defineTurnOf.angular.z = TURNRATE;
    } else if(sonarReadingSrv.response.readings.distance0 == UINT16_MAX){
        // It must be torwards distance 2 so we need to turn positive
        defineTurnOf.angular.z = TURNRATE;
    } else{
        // It must be towards distance 0 so we need to turn negative
        defineTurnOf.angular.z = TURNRATE * -1;
    }
}

// Thanks to how ROS was made, a massive chunk of this code has to be in main.
// Digusting.
int main(int argc, char **argv){
    ros::init(argc, argv, "assignment1/controller");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    ros::ServiceClient sonarReader = 
        nodeHandle.serviceClient<assignment1::getSonarReadings>(
            "assignment1/sonar_wrapper"
    );
    assignment1::getSonarReadings sonarReadingSrv;
    ros::Publisher driver =
        nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

#ifdef NOISY_SONAR
    ros::ServiceClient kalmanFilter = 
        nodeHandle.serviceClient<assignment1::kalmanEstimateVariance>(
            "assignment1/kalmanAlgorithm"
    );
#else
    assignment1::pidAlgorithm pidAlgorithmSrv;
    ros::ServiceClient PID_service = 
        nodeHandle.serviceClient<assignment1::pidAlgorithm>(
            "assignment1/pidAlgorithm"
    );
#endif
        while(ros::ok()){
            ros::spinOnce();
            if(!sonarReader.call(sonarReadingSrv)){
                ROS_ERROR("Failed to use assignment1/sonar_wrapper: is it running?");
                rate.sleep();
                continue;
            }
            geometry_msgs::Twist movement;
            if(sonarReadingSrv.response.readings.distance1 == UINT16_MAX){
                // We need to turn -> extract it as a function for cleanliness
                defineTurn(movement, sonarReadingSrv);
            } else{
                // We can start driving forwards
                pidAlgorithmSrv.request.distance = sonarReadingSrv.response.readings.distance1;
                if(!PID_service.call(pidAlgorithmSrv)){
                    ROS_ERROR("Failed to use assignment1/sonar_wrapper: is it running?");
                    rate.sleep();
                    continue;
                }
            }

            driver.publish(movement);
            rate.sleep();
        }

    return EXIT_SUCCESS;
}
