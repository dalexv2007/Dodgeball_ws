#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double limit)
        : kp_(kp), ki_(ki), kd_(kd), limit_(limit), integral_sum_(0.0), last_error_(0.0) {}

    double compute(double error, double dt) { //main output func
        double p_term = kp_ * error; // P = comparison of current error to desired state

        integral_sum_ += error * dt;
        integral_sum_ = std::clamp(integral_sum_, -10.0, 10.0); 
        double i_term = ki_ * integral_sum_; //I = sum of error over time

        double d_term = kd_ * ((error - last_error_) / dt); // D = current error derivative
        last_error_ = error;

        double output = p_term + i_term + d_term; // out = sum of all three terms
        return std::clamp(output, -limit_, limit_); // clamp output to max limit to prevent excessive commands
    }

    void reset() {
        integral_sum_ = 0.0;
        last_error_ = 0.0;
    }

private: 
    double kp_, ki_, kd_, limit_;
    double integral_sum_;
    double last_error_;
};

#endif