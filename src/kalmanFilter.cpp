#include <iostream>

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

class KalmanFilter{
    public:

    private:

};

int main(){
    std::cout << "Hello world!" << std::endl;
    return EXIT_SUCCESS;
}
