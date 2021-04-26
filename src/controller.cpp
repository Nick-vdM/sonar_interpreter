#include <inttypes.h>
#include "ros/ros.h"
#include "assignment1_setup/Sonars.h"
#include "assignment1/getSonarReadings.h"
#include "assignment1/pid_algorithm.h"
#include "assignment1/kalman_filter.h"
#include "gazebo_msgs/GetModelState.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include "geometry_msgs/Pose.h"
#include <vector>

// Maximum is 1.6 radians / second -> half a radian
// One degree = 180/M_PI
const double TURNRATE = M_PI/180 * 10; 
#ifdef NOISY_SONAR
uint32_t varianceSamples = 1000;
#endif

double euclideanDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
    ROS_INFO("Calculating euclid;model state: p1x %lf p1y %lf p1z %lf p2x %lf p2y %lf p3z %lf",
        p1.position.x, p1.position.y, p1.position.z, 
        p2.position.x, p2.position.y, p2.position.z);
    return sqrt(
        pow(p1.position.x - p2.position.x, 2) + 
        pow(p1.position.x - p2.position.y, 2) + 
        pow(p1.position.z - p2.position.z, 2)
    );
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

double calculateVariance(std::vector<uint16_t> & sonarReadings){
    ROS_INFO("Number of values collected %zu", sonarReadings.size());
    double mean{0.0};
    for(auto & v : sonarReadings){
        ROS_INFO("%" PRIu16, v);
        mean += v;
    }
    mean /= sonarReadings.size();
    ROS_INFO("%lf", mean);

    double variance{0};
    for(auto & v : sonarReadings){
        variance += (v - mean) * (v - mean);
    }
    variance /= sonarReadings.size();
    ROS_INFO("variance %lf", variance);
    return variance;
}

// Thanks to how ROS was made, a massive chunk of this code has to be in main.
// Sorry.
int main(int argc, char **argv){
    ros::init(argc, argv, "controller");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    assignment1::getSonarReadings sonarReadingSrv;
    ros::ServiceClient sonarReader = 
        nodeHandle.serviceClient<assignment1::getSonarReadings>(
            "sonar_wrapper"
    );
    ros::Publisher driver =
        nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

#ifdef NOISY_SONAR
    assignment1::kalman_filter kalmanFilterSrv;
    // specifically set variance to NaN now as a check for whether it's been
    // calculated later
    kalmanFilterSrv.request.R_i = std::numeric_limits<double>::quiet_NaN();
    ros::ServiceClient kalmanFilter = 
        nodeHandle.serviceClient<assignment1::kalman_filter>(
            "kalman_filter"
    );
    geometry_msgs::Pose lastPose;
    gazebo_msgs::GetModelState ModelStateSrv;
    ModelStateSrv.request.model_name = "turtlebot3_burger";
    ros::ServiceClient modelState = 
        nodeHandle.serviceClient<gazebo_msgs::GetModelState>(
            "/gazebo/get_model_state"
    );
    
#endif
    assignment1::pid_algorithm pidAlgorithmSrv;
    ros::ServiceClient PID_service = 
        nodeHandle.serviceClient<assignment1::pid_algorithm>(
            "pid_algorithm"
    );
    pidAlgorithmSrv.request.K_p = 1;
    pidAlgorithmSrv.request.K_i = 1;
    pidAlgorithmSrv.request.K_d = 1;
    pidAlgorithmSrv.request.lastError = 0;
    pidAlgorithmSrv.request.totalFValue = 0;
    pidAlgorithmSrv.request.T = 1000/100; // ms in a second / loop rate

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
            if(isnan(kalmanFilterSrv.request.R_i)){
                std::vector<uint16_t> sonarSamples;
                while(ros::ok() && varianceSamples > 0){
                    varianceSamples--;
                    if(varianceSamples % 100 == 0){
                        ROS_INFO("Collecting samples... %" PRIu32, varianceSamples);
                    }
                    ros::spinOnce();
                    if(!sonarReader.call(sonarReadingSrv)){
                        ROS_ERROR("Failed to use assignment1/sonar_wrapper: is it running?");
                        rate.sleep();
                        continue;
                    }
                    sonarSamples.push_back(sonarReadingSrv.response.readings.distance1);
                    rate.sleep();
                }
                kalmanFilterSrv.request.R_i = calculateVariance(sonarSamples);
                // R_0 = P_0
                kalmanFilterSrv.request.P_i_estimate = kalmanFilterSrv.request.R_i;
                ROS_INFO("R_i is %lf", kalmanFilterSrv.request.R_i);
                // since this is the first run z_i, y_i_estimate and
                // P_i_estimate should all be 0
                kalmanFilterSrv.request.z_i = 0;
                kalmanFilterSrv.request.y_i_estimate = 0;

                if(!modelState.call(ModelStateSrv)){
                    ROS_ERROR("Failed to use gazebo/get_model_state: is it running?");
                    rate.sleep();
                    continue;
                }
                lastPose = ModelStateSrv.response.pose;
            } else {
                // actually estimate z_i based on y_i using model states
                if(!modelState.call(ModelStateSrv)){
                    ROS_ERROR("Failed to use gazebo/get_model_state: is it running?");
                    rate.sleep();
                    continue;
                }
                lastPose = ModelStateSrv.response.pose;
                kalmanFilterSrv.request.y_i_estimate = euclideanDistance(lastPose, ModelStateSrv.response.pose);
                // y_i_estimate and P_i_estimate should already be handled
                // by previous loops
            }

            kalmanFilterSrv.request.z_i = sonarReadingSrv.response.readings.distance1;
            if(!kalmanFilter.call(kalmanFilterSrv)){
                ROS_ERROR("Failed to call kalman filter");
                rate.sleep();
                continue;
            }

            pidAlgorithmSrv.request.error = kalmanFilterSrv.response.y_i;
#else
            pidAlgorithmSrv.request.error = sonarReadingSrv.response.readings.distance1;
#endif
            ROS_INFO("Read error as %u", (unsigned int)pidAlgorithmSrv.request.error);
            if(!PID_service.call(pidAlgorithmSrv)){
                ROS_ERROR("Failed to use assignment1/sonar_wrapper: is it running?");
                rate.sleep();
                continue;
            }
            movement.linear.x = pidAlgorithmSrv.response.y;
        }

        driver.publish(movement);
        ROS_INFO("Published linear x: %lf angular z: %lf",
                    movement.linear.x, movement.angular.z);
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
