#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "dodgeball/msg/ball_location.hpp"
#include "dodgeball/pid_controller.hpp"
#include "nav_msgs/msg/odometry.hpp"

class DodgeballFSMNode : public rclcpp::Node {
public:
    // Constructor
    DodgeballFSMNode() : Node("dodgeball_fsm"), 
        current_state_(State::SEARCH),
        bearing_pid_(params_.bearing_kp, params_.bearing_ki, params_.bearing_kd, params_.bearing_limit),
        distance_pid_(params_.distance_kp, params_.distance_ki, params_.distance_kd, params_.distance_limit), 
        theta_pid_ (params_.theta_kp, params_.theta_ki, params_.theta_kd, params_.theta_limit) 
        {

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); //publisher for velocity

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->x_ = msg->pose.pose.position.x;
            this->y_ = msg->pose.pose.position.y;
            double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + 
                                        msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
            double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + 
                                            msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
            this->theta_ = std::atan2(siny_cosp, cosy_cosp);
        });

        ball_sub_ = this->create_subscription<dodgeball::msg::BallLocation>( //subscriber to ball location topic ball_finder
            "/ball_location", 10,
            [this](const dodgeball::msg::BallLocation::SharedPtr msg) { //
                this->ball_bearing_ = msg->bearing; //"bearing in this object = bearing in msg"
                this->ball_distance_ = msg->distance; //same with distance
                this->ball_found_ = msg->found; //same with found
            });
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DodgeballFSMNode::control_loop, this));
    }

private:
    // Tunable parameters - update values in real-time during testing for desired behavior
    struct TunableParams {
        // Movement parameters
        double search_rotation_speed = 0.5;  // rad/s - how fast to spin when searching
        double approach_distance_goal = 1.5; // meters - distance to stop at
        double kick_speed = 0.3;             // m/s - forward speed when kicking
        int kick_duration_ms = 3000;         // milliseconds - how long to kick
        
        // PID gains for bearing (angular velocity)
        double bearing_kp = 0.01, bearing_ki = 0.0, bearing_kd = 0.0;
        double bearing_limit = 1.0;          // max rad/s
        
        // PID gains for distance (linear velocity)
        double distance_kp = 0.3, distance_ki = 0.0, distance_kd = 0.0;
        double distance_limit = 0.3;         // max m/s

        //PID gains for theta
        double theta_kp = 1.0, theta_ki = 0.0, theta_kd = 0.0;
        double theta_limit = 1.0; // max rad/s
    } params_;

    struct VectorResult{
        double bearing;
        double distance;
    };

    static VectorResult get_vector(double x, double y, double target_x, double target_y) { // returns bearing and distance to target as VectorResult struct
        VectorResult result;
        double dx = target_x - x; // Calculate difference in x
        double dy = target_y - y; // Calculate difference in y
        result.bearing = std::atan2(dy, dx); // Convert to degrees
        result.distance = std::sqrt(dx * dx + dy * dy);
        return result;
    }

    static double normalize_angle(double angle) {
        while (angle >  M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void calculate_goals() {
        // Kick pos: 3m ahead along current heading (puts us 1.5m behind the ball)
        kick_x_ = x_ + 3.0 * std::cos(theta_);
        kick_y_ = y_ + 3.0 * std::sin(theta_);

        // Intermediate: arc to the side to get around the ball
        double inter_dist  = 3.0 * std::sqrt(2.0) / 2.0;
        double inter_angle = theta_ + M_PI / 4.0; // left side; change + to - for right
        inter_x_ = x_ + inter_dist * std::cos(inter_angle);
        inter_y_ = y_ + inter_dist * std::sin(inter_angle);
    }

    enum class State { SEARCH, APPROACH, NAV_TO_INTER, NAV_TO_KICK_POS, LINE_UP, KICK };
    State current_state_;

    double ball_bearing_ = 0.0; // hold targeting data
    double ball_distance_ = 0.0;
    bool ball_found_ = false;
    double image_center_x_ = 125.0;
    double dt = 0.1; // Assuming control loop runs every 100ms
    double inter_x_ = 0.0, inter_y_ = 0.0;  // world position of intermediate goal
    double kick_x_  = 0.0, kick_y_  = 0.0;  // world position behind the ball
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0; //current position and orientation of the robot

    PIDController bearing_pid_;
    PIDController distance_pid_;
    PIDController theta_pid_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; //velo pub
    rclcpp::Subscription<dodgeball::msg::BallLocation>::SharedPtr ball_sub_; //ball lo sub
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; //odom sub
    rclcpp::TimerBase::SharedPtr timer_; //timer for control loop
    rclcpp::Time kick_start_time_; //time point for kick timing

    void control_loop() {
        geometry_msgs::msg::Twist twist_cmd; // pub'd twist msg as twist_cmd
        double dist_error = std::abs(params_.approach_distance_goal - ball_distance_); //calculate distance error for PID
        double bearing_error = std::abs(image_center_x_ - ball_bearing_); //calculate bearing error for PID

        switch (current_state_) { //switch for FSM
            case State::SEARCH: 
                twist_cmd.linear.x = 0.0; //ensure no forward movement
                twist_cmd.angular.z = params_.search_rotation_speed; //rotate in place
                if(ball_found_) {
                    twist_cmd.angular.z = 0.0; //stop rotating
                    current_state_ = State::APPROACH; //switch to approach state when ball found
                    RCLCPP_INFO(this->get_logger(), "SEARCH -> APPROACH");
                } 
                break;

            case State::APPROACH:
                // PID compute returns the control output directly
                twist_cmd.angular.z = bearing_pid_.compute(image_center_x_ - ball_bearing_, dt);
                twist_cmd.linear.x = distance_pid_.compute(ball_distance_ - params_.approach_distance_goal, dt);
                
                if(!ball_found_){
                    current_state_ = State::SEARCH; //if ball lost, go back to search
                    RCLCPP_INFO(this->get_logger(), "APPROACH -> SEARCH");
                }
                else if (dist_error < 0.3 && bearing_error < 20.0) { //if within approach distance, switch to kick
                    twist_cmd.linear.x = 0.0; //stop forward movement
                    twist_cmd.angular.z = 0.0; //stop rotation
                    calculate_goals(); //calculate intermediate and kick positions based on current odom
                    current_state_ = State::NAV_TO_INTER; //go to intermediate point to get around ball
                    RCLCPP_INFO(this->get_logger(), "APPROACH -> NAV_TO_INTER");
                }
                break;

            case State::NAV_TO_INTER:
            {
                VectorResult vec = get_vector(x_, y_, inter_x_, inter_y_); //get bearing and distance to intermediate point
                double theta_error = normalize_angle(theta_ - vec.bearing); //compute angle to intermediate point and control output
                twist_cmd.linear.x = 0.15; //constant forward speed
                twist_cmd.angular.z = theta_pid_.compute(theta_error, dt); //angular speed to turn towards intermediate point
            
                if(vec.distance < 0.5){//if close to intermediate point, switch to nav to kick pos
                    current_state_ = State::NAV_TO_KICK_POS;
                    RCLCPP_INFO(this->get_logger(), "NAV_TO_INTER -> NAV_TO_KICK_POS");
                }
                break;            
            }

            case State::NAV_TO_KICK_POS:
            {
                VectorResult vec = get_vector(x_, y_, kick_x_, kick_y_); //get bearing and distance to kick position
                double theta_error = normalize_angle(theta_ - vec.bearing); //compute angle to kick position and control output
                twist_cmd.linear.x = 0.15; //constant forward speed
                twist_cmd.angular.z = theta_pid_.compute(theta_error, dt); //angular speed to turn towards kick pos

                if(vec.distance < 1.0){ //if close to kick position, switch to line up
                    current_state_ = State::LINE_UP;
                    RCLCPP_INFO(this->get_logger(), "NAV_TO_KICK_POS -> LINE_UP");
                }
                break;
            }

            case State::LINE_UP:
            {
                twist_cmd.angular.z = bearing_pid_.compute(image_center_x_ - ball_bearing_, dt); //rotate to line up with ball
                twist_cmd.linear.x = 0.0; //dont move forward, just rotate in place
                bearing_error = std::abs(image_center_x_ - ball_bearing_); //update bearing error to check if well aligned with ball

                if(bearing_error < 10.0) { //if well aligned with ball, switch to kick
                    twist_cmd.angular.z = 0.0; //stop rotating
                    twist_cmd.linear.x = 0.0; //ensure no forward movement
                    kick_start_time_ = this->now(); //record time when kick starts for timing the kick duration
                    current_state_ = State::KICK;
                    RCLCPP_INFO(this->get_logger(), "LINE_UP -> KICK");
                }
                break;
            }

            case State::KICK:
                twist_cmd.angular.z = 0.0; //no rotation while kicking
                twist_cmd.linear.x = params_.kick_speed; //move forward at kick speed
                if((this->now() - kick_start_time_).seconds() >= params_.kick_duration_ms / 1000.0) { //after kick duration, go back to search
                    twist_cmd.linear.x = 0.0; //stop movement
                    current_state_ = State::SEARCH;
                    RCLCPP_INFO(this->get_logger(), "KICK -> SEARCH");
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
    rclcpp::spin(std::make_shared<DodgeballFSMNode>());
    rclcpp::shutdown();
    return 0;
}