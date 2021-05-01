/*
 * By Nick van der Merwe - s5151332
 */
#include <limits>
#include <gazebo_msgs/ModelStates.h>
#include "ros/ros.h"
#include "assignment1/kalman_filter.h"

/*
 * z_i is sensor reading
 * y_i^- is estimated value at i
 * P_i^- is estimated variance at i
 * R_i is sensor variance
 * === Formulas ===
 * K_i = P_i^- / (P_i^- + R_i)
 * y_i = y_i^- + K_i(z_i-y_i^-)
 * 
 * P_i = (1-K_i) * P_i^-
 * y_0 = z_0, P_0 = R_0
 * 
 * Grab some values in, work out 
 * the variance and calculate the rest. 
 * Base the variance off a normal distribution
 * 
 * y_i(0) = 0
 * P_i^- = 0
 * K_i(0) = 0
 * y_i(0) = z
 * P_0 = R_0 (R_i is constant)
 * 
 * y_i is based on previous movement
 * so use get_model_state between last 
 * position and current position <permission from Morgan
 * at 1:08 week 4 lab. In reality this also needs variance
 * readings and etc
 * 
 * Also, for this assignment P_i^- 
 * does not change at all just like R_i
 * 
 * We are returning P_i
 */

bool kalmanFilter(
        assignment1::kalman_filter::Request &req,
        assignment1::kalman_filter::Response &res){
        double K_i = req.P_i_estimate / (req.P_i_estimate + req.R_i);
        res.y_i = req.y_i_estimate + K_i*(req.z_i - req.y_i_estimate);
        ROS_INFO("P_i_estimate (before) %lf", req.P_i_estimate);
        res.P_i = (1 - K_i) * req.P_i_estimate;
        ROS_INFO("K_i %lf R_I %lf z_i %u y_i_estimate %lf P_i_estimate(after) %lf",
                K_i, req.R_i, req.z_i, req.y_i_estimate, res.P_i);
        
        ROS_INFO("Returning y_i as %lf", res.y_i);
        return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "kalman_filter_node");
    ros::NodeHandle nodeHandle;

    ros::ServiceServer findFilteredOutput = 
        nodeHandle.advertiseService("kalman_filter", kalmanFilter);

    ROS_INFO("Ready to manage requests");
    ros::spin();

    return EXIT_SUCCESS;
}
