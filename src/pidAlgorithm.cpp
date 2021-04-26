/*
 * Written by Nick van der Merwe - s5151332
 * 
 */
#include "ros/ros.h"
#include "assignment1_setup/Sonars.h"
#include "assignment1/pid_algorithm.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <algorithm>



/*
* We want to calculate the PID. To increase readability we 
* are going to split each of these formulas into a separate function:
* y(t_i) = P(t_i) + I(t_i) + D(t_i)
* P(t_i) = K_p*e(t_i)
* f(t_i) = e(t_i) + f(t_{i-1})
* I(t_i) = K_i*f(t_i)
* D(t_i) = K_d * ((e(t_i)-e(t_{i-1)})/T)
* 
* e(t) = distance from object at t
* T = latency
*/
bool pidAlgorithm(
        assignment1::pid_algorithm::Request & req,
        assignment1::pid_algorithm::Response &res){
    double P = req.K_p * req.error;
    res.totalFValue = req.error * req.T + req.totalFValue;
    double I = req.K_i * res.totalFValue;
    // split D into top and bottom so its more readable
    double && topD = req.error - req.lastError;
    double D = req.K_d * (topD / req.T);

    res.y = P + I + D;
    // checking the documentation 22.0 is the maximum speed
    res.y = std::min(res.y, 0.22);
    res.y = std::max(res.y, 0.0);

    ROS_INFO("lastError: %u totalFValue %lf", 
            (unsigned int) res.lastError, res.totalFValue);
    ROS_INFO("K_p: %lf K_i %lf K_d %lf", req.K_p, req.K_i, req.K_d);
    ROS_INFO("P: %lf, I:, %lf, D: %lf", P,I, D);
    ROS_INFO("Returning y as %lf", res.y);
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pid_algorithm_node");
    ros::NodeHandle nodeHandle;

    ros::ServiceServer getSonarReadings =
        nodeHandle.advertiseService("pid_algorithm", pidAlgorithm);

    ROS_INFO("Ready to manage requests");
    ros::spin();

    return EXIT_SUCCESS;
}
