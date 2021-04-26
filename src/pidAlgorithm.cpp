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




class PIDCalculator{
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
    public:
        PIDCalculator() = default;

        bool calcVelocity(
                assignment1::pidAlgorithm::Request &req,
                assignment1::pidAlgorithm::Response &res){
                    // this is y(t_i) -> req.distance=e(t_i)
                    // in other words, distance is actually error
            res.velocity = this->P(req.distance) + this->I(req.distance) + this->D(req.distance);
            // checking the documentation 22.0 is the maximum speed
            res.velocity = std::max(res.velocity, 22.0);
            res.velocity = std::min(res.velocity, 0.0);
            return true;
        }

    private:
        // Because t_0 = 0, these just start at 0
        double lastError{0};
        double lastFValue{0};

        const double K_p = 1;
        const double K_i = 1;
        const double K_d = 1;

        const uint16_t RATE = 100; // loops per second in sonars.cc
        const double T = RATE / 1000; // ms 

        // Changing distance -> e_t so that it makes more sense
        // with the formulas
        double P(uint16_t e_t){
            return this->K_p * e_t;
        }

        double f(u_int16_t e_t){
            lastFValue = (this->T * e_t) + lastFValue;
            return lastFValue;
        }

        double I(uint16_t e_t){
            return this->K_i*this->f(e_t);
        }

        double D(uint16_t e_t){
            // this one is annoying to read so split it apart
            double top = e_t - this->lastError;
            return this->K_d * (top/this->T);
        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "assignment1/pid_algorithm");
    ros::NodeHandle nodeHandle;

    PIDCalculator pid{};

    ros::ServiceServer getSonarReadings =
        nodeHandle.advertiseService("pidAlgorithm", 
                &PIDCalculator::calcVelocity, &pid);

    return EXIT_SUCCESS;
}
