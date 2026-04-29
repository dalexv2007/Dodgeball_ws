#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_puppy/msg/ball_location.hpp"
#include "robot_puppy/pid_controller.hpp"

class RobotPuppyNode : public rclcpp::Node {
public:
    // Constructor
    RobotPuppyNode() : Node("robot_puppy"), 
        current_state_(State::SEARCH),
        bearing_pid_(params_.bearing_kp, params_.bearing_ki, params_.bearing_kd, params_.bearing_limit),
        distance_pid_(params_.distance_kp, params_.distance_ki, params_.distance_kd, params_.distance_limit) {

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); //publisher for velocity
        ball_sub_ = this->create_subscription<robot_puppy::msg::BallLocation>( //subscriber to ball location topic ball_finder
            "ball_location", 10,
            [this](const robot_puppy::msg::BallLocation::SharedPtr msg) { //
                this->ball_bearing_ = msg->bearing; //"bearing in this object = bearing in msg"
                this->ball_distance_ = msg->distance; //same with distance
                this->ball_found_ = msg->found; //same with found
            });
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotPuppyNode::control_loop, this));
    }

private:
    // Tunable parameters - update values in real-time during testing for desired behavior
    struct TunableParams {
        // Movement parameters
        double search_rotation_speed = 0.5;  // rad/s - how fast to spin when searching
        double approach_distance_goal = 1.0; // meters - distance to stop at
        double kick_speed = 0.3;             // m/s - forward speed when kicking
        int kick_duration_ms = 3000;         // milliseconds - how long to kick
        
        // PID gains for bearing (angular velocity)
        double bearing_kp = 0.01, bearing_ki = 0.0, bearing_kd = 0.0;
        double bearing_limit = 1.0;          // max rad/s
        
        // PID gains for distance (linear velocity)
        double distance_kp = 0.5, distance_ki = 0.0, distance_kd = 0.0;
        double distance_limit = 0.5;         // max m/s
    } params_;

    enum class State { SEARCH, APPROACH, KICK };
    State current_state_;

    double ball_bearing_ = 0.0; // hold targeting data
    double ball_distance_ = 0.0;
    bool ball_found_ = false;
    double bearing_goal_ = 125;
    double dt = 0.1; // Assuming control loop runs every 100ms

    PIDController bearing_pid_; // PID controller for bearing
    PIDController distance_pid_; // PID controller for distance

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; //velo pub
    rclcpp::Subscription<robot_puppy::msg::BallLocation>::SharedPtr ball_sub_; //ball lo sub
    rclcpp::TimerBase::SharedPtr timer_; //timer for control loop
    rclcpp::Time kick_start_time_; //time point for kick timing

    void control_loop() {
        geometry_msgs::msg::Twist twist_cmd; // pub'd twist msg as twist_cmd
        double dist_error = std::abs(params_.approach_distance_goal - ball_distance_); //calculate distance error for PID
        double bearing_error = std::abs(bearing_goal_ - ball_bearing_); //calculate bearing error for PID

        switch (current_state_) { //switch for FSM
            case State::SEARCH: 
                twist_cmd.linear.x = 0.0; //ensure no forward movement
                twist_cmd.angular.z = params_.search_rotation_speed; //rotate in place
                if(ball_found_) {
                    twist_cmd.angular.z = 0.0; //stop rotating
                    current_state_ = State::APPROACH; //switch to approach state when ball found
                } 
                break;

            case State::APPROACH:
                // PID compute returns the control output directly
                twist_cmd.angular.z = bearing_pid_.compute(bearing_goal_ - ball_bearing_, dt);
                twist_cmd.linear.x = distance_pid_.compute(params_.approach_distance_goal - ball_distance_, dt);
                
                if(!ball_found_){
                    current_state_ = State::SEARCH; //if ball lost, go back to search
                }
                else if (dist_error < 0.1 && bearing_error < 15.0) { //if within approach distance, switch to kick
                    twist_cmd.linear.x = 0.0; //stop forward movement
                    twist_cmd.angular.z = 0.0; //stop rotation
                    current_state_ = State::KICK;
                    kick_start_time_ = this->now(); //record the time we started kicking
                }
                break;

            case State::KICK:
                twist_cmd.angular.z = 0.0; //no rotation while kicking
                twist_cmd.linear.x = params_.kick_speed; //move forward at kick speed
                if((this->now() - kick_start_time_).seconds() >= params_.kick_duration_ms / 1000.0) { //after kick duration, go back to search
                    twist_cmd.linear.x = 0.0; //stop movement
                    current_state_ = State::SEARCH;
                }
                break;
        }

        // Broadcast the motor command to the bus
        cmd_vel_pub_->publish(twist_cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // Spin acts as the infinite loop, keeping the node alive to listen for callbacks
    rclcpp::spin(std::make_shared<RobotPuppyNode>());
    rclcpp::shutdown();
    return 0;
}