/*
 * Written by Nick van der Merwe - s5151332
 * 
 */
#include "ros/ros.h"
#include "assignment1_setup/Sonars.h"
#include "assignment1/pidAlgorithm.h"
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
        assignment1::pidAlgorithm::Request &req,
        assignment1::pidAlgorithm::Response &res){
    double P = req.K_p * req.error;
    req.totalFValue = req.error * req.T + req.totalFValue;
    double I = req.K_i * req.totalFValue;
    // split D into top and bottom so its more readable
    double && topD = req.error - req.lastError;
    double D = req.K_d * (topD / req.T);

    res.velocity = P + I + D;
    // checking the documentation 22.0 is the maximum speed
    res.velocity = std::max(res.velocity, 0.22);
    res.velocity = std::min(res.velocity, 0.0);

    // just update last error to be the current error
    res.lastError = res.error;
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "assignment1/pid_algorithm");
    ros::NodeHandle nodeHandle;

    PIDCalculator pid{};

    ros::ServiceServer getSonarReadings =
        nodeHandle.advertiseService("pidAlgorithm", pidAlgorithm);

    return EXIT_SUCCESS;
}
