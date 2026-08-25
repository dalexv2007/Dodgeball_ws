#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double limit)
        : kp_(kp), ki_(ki), kd_(kd), limit_(limit), integral_sum_(0.0), last_error_(0.0), has_previous_error_(false) {} //construct with internal values with received values.

    
    double compute(double error, double dt) { //main output func. Receives signed error between current and desired states.
        double p_term = kp_ * error; // P for Proportion: comparing current state to desired state directly.

        integral_sum_ += error * dt; // accumulated error over time across multiple calls to compute(). This is the integral term.  
        integral_sum_ = std::clamp(integral_sum_, -10.0, 10.0); //clamp so accumulated error doesn't grow to infinity.
        double i_term = ki_ * integral_sum_; // I for Integral: accumulated error over time.

        double d_term = 0.0;
        if(has_previous_error_){
            d_term = kd_ * ((error - last_error_) / dt); // D for Derivative: describes how error is changing over time.
        }


        last_error_ = error; //together, P, I, and D are used to compute a value that represents the best guess for how to correct the error.
        has_previous_error_ = true; //set flag to true so derivative term can be calculated next time compute() is called.
        
        double output = p_term + i_term + d_term; // out = sum of all three terms
        return std::clamp(output, -limit_, limit_); // clamp output to max limit to prevent excessive commands
    }

    void reset() { //function to reset internal accumulated values.
        integral_sum_ = 0.0;
        last_error_ = 0.0;
        has_previous_error_ = false;
    }

private: 
    double kp_, ki_, kd_, limit_;
    double integral_sum_;
    double last_error_;
    bool has_previous_error_;
};

#endif

/*  Troubleshooting notes
*   Inputs:
*   - Get image -> process image -> get ball location as (bearing, distance) -> send to PID to compute error
*   In Dodgeball_FSM.pp, image processing needs to be reviewed. 
*   std::abs in control_loop() is BAD, as we need signed errors for PID to work properly.
*   NOT NECESSARILY: state transition checks are not the same as PID inputs, go through Dodgeball_FSM.cpp and organize PID inputs versus state transition checks.
*   Later: calculate dt from time stamps rather than hardcoded. 
*
*   Solutions:
*   - BallFinder.py bearing calc fixed, check correct published to ball_location.bearing.
*   - added has_previous_error_ flag to PIDController to ensure derivative term is only calculated after first call to compute().
*   - 
*/