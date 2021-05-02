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
const double TURNRATE = M_PI / 180 * 50;
#ifdef NOISY_SONAR
uint32_t varianceSamples = 1000;
#endif

double euclideanDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    ROS_INFO("Calculating euclid;model state: p1x %lf p1y %lf p1z %lf p2x %lf p2y %lf p3z %lf",
             p1.position.x, p1.position.y, p1.position.z,
             p2.position.x, p2.position.y, p2.position.z);
    return sqrt(
            pow(p1.position.x - p2.position.x, 2) +
            pow(p1.position.y - p2.position.y, 2)
    );
}

void defineTurn(geometry_msgs::Twist &defineTurnOf,
                assignment1::getSonarReadings sonarReadingSrv) {
    // We need to rotate
    if (sonarReadingSrv.response.readings.distance0 == UINT16_MAX && 
        sonarReadingSrv.response.readings.distance2 == UINT16_MAX) {
        // Unable to see where the robot is so just turn right
        defineTurnOf.angular.z = TURNRATE * -1;
    } else if (sonarReadingSrv.response.readings.distance0 == UINT16_MAX) {
        // It must be torwards distance 2 so we need to turn negative
        defineTurnOf.angular.z = TURNRATE * -1;
    } else {
        // It must be towards distanCe 0 so we need to turn positive
        defineTurnOf.angular.z = TURNRATE;
    }
}

double calculateVariance(std::vector <uint16_t> &sonarReadings) {
    ROS_INFO("Number of values collected %zu", sonarReadings.size());
    double mean{0.0};
    for (auto &v : sonarReadings) {
        ROS_INFO("%" PRIu16, v);
        mean += v;
    }
    mean /= sonarReadings.size();
    ROS_INFO("%lf", mean);

    double variance{0};
    for (auto &v : sonarReadings) {
        variance += (v - mean) * (v - mean);
    }
    variance /= sonarReadings.size() - 1;
    ROS_INFO("variance %lf", variance);
    return variance;
}

// Reading this would be incredibly painful because a lot of sections cannot
// be extracted thanks to how spin() must be in main. As a workaround,
// check the psuedo code in the report first before trying to understand this.
int main(int argc, char **argv) {
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
    pidAlgorithmSrv.request.K_p = 0.22/100.0;
    ROS_INFO("Initial K_p is %lf", pidAlgorithmSrv.request.K_p);
    pidAlgorithmSrv.request.K_i = 0;
    pidAlgorithmSrv.request.K_d = -0.2;
    pidAlgorithmSrv.request.lastError = 0;
    pidAlgorithmSrv.request.totalFValue = 0;
    pidAlgorithmSrv.request.T = 10 / 1000; // convert 10ms to seconds
    bool firstPid = true;
    bool turning = false;
    // Quickly publish a zero movement twist in case the last program terminated
    // with the robot moving
    driver.publish(geometry_msgs::Twist{});
    ros::spinOnce();
    rate.sleep();

    while (ros::ok()) {
        // Would love to extract all of these node client calls so they do not
        // take up five lines each, but the continue function is in them...
        if (!sonarReader.call(sonarReadingSrv)) {
            ROS_ERROR("Failed to use assignment1/sonar_wrapper: is it running?");
            rate.sleep();
            continue;
        }
        geometry_msgs::Twist movement;
        if (sonarReadingSrv.response.readings.distance1 == UINT16_MAX) {
            // It could be a dud, if we are not already turning check whether
            // it happens another five times in a row
            if(!turning){
                // publish zero movement
                driver.publish(movement);
                for(int i = 0; i < 5; i++){
                    // Test if we get the same reading five times
                    rate.sleep();
                    ros::spinOnce();
                    if (!sonarReader.call(sonarReadingSrv)) {
                        ROS_ERROR("Failed to use assignment1/sonar_wrapper: is it running?");
                        rate.sleep();
                        continue;
                    }
                    if(sonarReadingSrv.response.readings.distance1 != UINT16_MAX){
                        continue;
                    }
                // Enter turning mode
                turning = true;
                }
            }
            defineTurn(movement, sonarReadingSrv);
            } else {
                // We can start driving forwards
                if(turning){
                    // quickly publish a zero movement to stop moving
                    turning = false;
                    driver.publish(movement);
                    rate.sleep();
                    continue;
                }
#ifdef NOISY_SONAR
            // just go ahead and generate the variance now
            // Can't extract method because we use spin
            if(isnan(kalmanFilterSrv.request.R_i)){
                std::vector<uint16_t> sonarSamples;
                while(ros::ok() && varianceSamples > 0){
                    varianceSamples--;
                    if(varianceSamples % 100 == 0){
                        ROS_INFO("Collecting samples for variance calculation... %" PRIu32 " left", varianceSamples);
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
                // since this is the first run z_i, y_i_estimate and
                // P_i_estimate = 0. As a side effect this causes
                // P_0 = R_0 as P_i=(1-K)*P_i_estimate and K = 0 when
                // P_i_estimate = 0.
                kalmanFilterSrv.request.z_i = 0;
                kalmanFilterSrv.request.y_i_estimate = 0;
                kalmanFilterSrv.request.P_i_estimate = 
                    kalmanFilterSrv.request.R_i;

                if(!modelState.call(ModelStateSrv)){
                    ROS_ERROR("Failed to use gazebo/get_model_state: is it running?");
                    rate.sleep();
                    continue;
                }
                lastPose = ModelStateSrv.response.pose;
            } else {
                // Variance has been calculated, do the actual filter instead
                if(!modelState.call(ModelStateSrv)){
                    ROS_ERROR("Failed to use gazebo/get_model_state: is it running?");
                    rate.sleep();
                    continue;
                }
                // Sonars makes the units into centimetres instead so follow
                // suite 
                double && euclidean = euclideanDistance(lastPose, ModelStateSrv.response.pose) * 100;
                ROS_INFO("Euclidean distance is %lf", euclidean);
                kalmanFilterSrv.request.y_i_estimate =
                        kalmanFilterSrv.response.y_i - euclidean;

                lastPose = ModelStateSrv.response.pose;
            }

            kalmanFilterSrv.request.z_i = sonarReadingSrv.response.readings.distance1;
            if(!kalmanFilter.call(kalmanFilterSrv)){
                ROS_ERROR("Failed to call kalman filter");
                rate.sleep();
                continue;
            }
            // Also make P_i_estimate = P_i for the next run
            ROS_INFO("P_i_estimate (before) = %lf, P_i = %lf", kalmanFilterSrv.request.P_i_estimate, kalmanFilterSrv.response.P_i);
            kalmanFilterSrv.request.P_i_estimate = 
                kalmanFilterSrv.response.P_i;
            ROS_INFO("P_i_estimate (after) = %lf", kalmanFilterSrv.request.P_i_estimate);

            pidAlgorithmSrv.request.error = kalmanFilterSrv.response.y_i;
#else
            pidAlgorithmSrv.request.error = (double)sonarReadingSrv.response.readings.distance1;
#endif
            ROS_INFO("Read error as %lf", pidAlgorithmSrv.request.error);
            ROS_INFO("Calling PID_service with K_p = %lf", pidAlgorithmSrv.request.K_p);
            if (!PID_service.call(pidAlgorithmSrv)) {
                ROS_ERROR("Failed to use assignment1/pid_algorithm: is it running?");
                rate.sleep();
                continue;
            }
            movement.linear.x = pidAlgorithmSrv.response.y;
            // Reset PID's requests based on the responses for the next run
            pidAlgorithmSrv.request.lastError =
                    pidAlgorithmSrv.request.error;
            pidAlgorithmSrv.request.totalFValue =
                    pidAlgorithmSrv.response.totalFValue;
        }

        driver.publish(movement);
        ROS_INFO("Published linear x: %lf angular z: %lf",
                 movement.linear.x, movement.angular.z);
        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
