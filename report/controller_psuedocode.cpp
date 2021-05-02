//
// Created by nicol on 1/05/2021.
// https://www.linkedin.com/in/nick-vd-merwe/
// nick.jvdmerwe@gmail.com

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
        defineTurnOf.angular.z = TURNRATE * 1;
    } else {
        // It must be towards distanCe 0 so we need to turn positive
        defineTurnOf.angular.z = TURNRATE;
    }
}

// this point onwards is psuedo code
void noisySteps(){
    // EVERYTHING is just global here. This function is in main in the actual code
    if(variance is not a number){
        calculateVariance();
        set z_i, y_i_estimate, P_i_estimate = 0;
        call modelState;
        set lastPose = modelState.response.pose;
    } else { // variance is known -> find y_i_estimate
        call model state service;
        calculate y_i_estimate using euclid;
        // Since we would have done the other branch in the first run
        // we do this before calling the kalman filter service
        set kalmanFilterSrv.request_P_i_estimate =
                    kalmanFilterSrv.response.P_i;
    }
    call kalmanFilter;
    pidAlgorithm.request.error = kalmanFilterSrv.response.y_i;
}

void defineMovement(){
    // EVERYTHING is just global here. This function is in main in the actual code
    if(noisy){
        noisySteps();
    } else{
        pidAlgorithm.request.error =
                sonarReadingSrv.response.readings.distance1;
    }
    call PID_service;
    movement.linear.x = pidAlgorithmSrv.response.y;
    pidAlgorithmSrv.request.lastError =
            pidAlgorithmSrv.request.error;
    pidAlgorithmSrv.request.totalFValue =
            pidAlgorithmSrv.response.totalFValue;
}

int main(){
    init ros_node;
    ros::Rate rate = 100;

    define sonarReader (sonar wrapper service client);
    define driver (publisher for cmd_vel);
    if(noisy){
        define kalmanFilter;
        declare lastPose;
        declare ModelStateSrv (storage for modelState);
        define ModelStateSrv.model_name = "turtlebot3_burger";
        deifine modelState (service client);
    }
    declare pidAlgorithmSrv (storage for PID_service);
    define PID_service (node client for PID):
    definePIDSrvInitialValues(pidAlgorithmSrv);
    bool firstPid = true;

    // This is the important section
    while(ros::ok()){
        spin once;
        call sonarReadingSrv

        declare Twist movement;
        if(we are not facing the bowl){
            defineTurn();
        } else{
            defineMovement();
        }
        // publish either the turn or the movement
        driver.publish(movement);
        rate.sleep();
    }
}
