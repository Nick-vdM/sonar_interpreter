#include "ros/ros.h"
#include "assignment1_setup/Sonars.h"
#include "assignment1/getSonarReadings.h"
#include "assignment1/pid_algorithm.h"
#include "assignment1/kalman_filter.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

// Maximum is 1.6 radians / second -> half a radian
// One degree = 180/M_PI
const double TURNRATE = 180/M_PI; 
#ifdef NOISY_SONAR
uint32_t varianceSamples = 1000;
#endif

double euclideanDistance(geometry_msgs::Pose p1, geometry_msgs::p2){
    return sqrt(
        pow(p1.x - p2.x, 2) + 
        pow(p1.y - p2.y, 2) + 
        pow(p1.z - p2.z, 2) + 
    )
}

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

void calculateVariance(vector<uint16_t> sonarReadings){
    double mean{0.0};
    for(auto & v : sonarReadings){
        mean += v / sonarReadings.size();
    }

    double variance{0};
    for(auto & v : sonarReadings){
        variance += (v - mean) * (v - mean);
    }
    return variance;
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
    assignment1::kalman_filter kalmanFilterSrv;
    // specifically set variance to NaN now as a check for whether it's been
    // calculated later
    kalmanFilterSrv.req.R_i = numeric_limits<double>::quiet_NaN();
    ros::ServiceClient kalmanFilter = 
        nodeHandle.serviceClient<assignment1::kalman_filter>(
            "assignment1/kalmanAlgorithm"
    );
    geometry_msgs::Pose lastPose;
    gazebo_msgs::GetModelState ModelStateSrv;
    ros::ServiceClient modelState = 
        nodeHandle.serviceClient<gazebo_msgs::GetModelState>(
            "/gazebo/get_model_state"
    );
    
#else
    assignment1::pid_algorithm pidAlgorithmSrv;
    ros::ServiceClient PID_service = 
        nodeHandle.serviceClient<assignment1::pid_algorithm>(
            "assignment1/pid_algorithm"
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
#ifdef NOISY_SONAR
                // just go ahead and generate the variance now
                if(isnan(kalmanFilterSrv.req.R_i)){
                    vector<uint16_t> sonarSamples;
                    while(ros::ok() && varianceSamples > 0){
                        VARIANCE_SAMPLES--;
                        ros::spinOnce();
                        if(!sonarReader.call(sonarReadingSrv)){
                            ROS_ERROR("Failed to use assignment1/sonar_wrapper: is it running?");
                            rate.sleep();
                            continue;
                        }
                        sonarSamples.push_back(sonarSamples.response)
                        rate.sleep();
                    }
                    kalmanFilterSrv.req.R_i = calculateVariance(sonarSamples);
                    // since this is the first run z_i, y_i_estimate and
                    // P_i_estimate should all be 0
                    kalmanFilterSrv.req.z_i = 0;
                    kalmanFilterSrv.req.y_i_estimate = 0;
                    kalmanFilterSrv.req.P_i_estimate = 0;

                    if(!modelState.call(ModelStateSrv)){
                        ROS_ERROR("Failed to use gazebo/get_model_state: is it running?");
                        rate.sleep();
                        continue;
                    }
                    lastPose = ModelStateSrv.response.position;
                } else {
                    // actually estimate z_i based on y_i using model states
                    if(!modelState.call(ModelStateSrv)){
                        ROS_ERROR("Failed to use gazebo/get_model_state: is it running?");
                        rate.sleep();
                        continue;
                    }
                    req.z_i = lastPose;
                    lastPose = euclideanDistance(lastPose, ModelStateSrv.response.position);
                    // y_i_estimate and P_i_estimate should already be handled
                    // by previous loops
                }

                if(!kalmanFilter.call(kalmanFilter)){
                    ROS_ERROR("Failed to call kalman filter");
                    rate.sleep;
                    continue;
                }

                pidAlgorithmSrv.request.distance = kalmanFilter.res.y_i;
#else
                pidAlgorithmSrv.request.distance = sonarReadingSrv.response.readings.distance1;
#endif
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
