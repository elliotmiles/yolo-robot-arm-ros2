//This script has FIXED Z-HEIGHT, and horizontal L3 

//phi is the base angle, 0 is when the limit switch is directly opposite home [-180, 180]
//theta1 is the angle between L1 and the horizontal
//theta2 is the angle between L1 and L2
//theta3 is the angle between L2 and L3

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <vector>
#include <array>
#include <cmath>


class RobotArm
{
public:
    RobotArm(double L1, double L2, double L3, double target_z) : L1_(L1), L2_(L2), L3_(L3), target_z_(target_z) {}
 

    std::array<double, 2> wrist_pos(std::vector<double>& target_pos) {
        double x = target_pos[0];
        double y = target_pos[1];

        double target_r = sqrt(x*x + y*y);

        double maxLen = L1_ + L2_ + L3_;
        if (maxLen < target_r) {
            std::cerr << "Target is out of reach." << std::endl;
            return {0.0, 0.0}; 
        } 

        double r = target_r - L3_;

        return {r, target_z_};
    }

    double normalise_angle(double angle) {
    return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    std::array<double, 4> inverse_kinematics(std::vector<double>& target_pos) {
        double x = target_pos[0];
        double y = target_pos[1];

        // base rotation
        double phi = atan2(y, x);

        double theta_1;
        double theta_2;
        double theta_3;

        // wrist position
        auto coords = wrist_pos(target_pos);

        // theta 2
        double cos_theta_2 = (coords[0]*coords[0] + coords[1]*coords[1] - L1_*L1_ - L2_*L2_) / (2 * L1_ * L2_);
        if (cos_theta_2 < -1 || cos_theta_2 > 1) 
        {
            std::cerr << "Target is out of reach." << std::endl;
            return {0.0, 0.0, 0.0, 0.0}; 
        }
        else 
        {
            theta_2 = acos(cos_theta_2);
        }

        // theta 1
        double alpha = atan2(coords[1], coords[0]);
        double beta = asin((L2_ * sin(theta_2)) / sqrt(coords[0]*coords[0] + coords[1]*coords[1]));
        theta_1 = alpha + beta;

        // theta 3
        theta_3 = 2*M_PI - (theta_1 + (M_PI - theta_2));

        // normalise angles to [-pi, pi]
        theta_1 = normalise_angle(theta_1);
        theta_2 = normalise_angle(theta_2);
        theta_3 = normalise_angle(theta_3);

        return {phi, theta_1, M_PI - theta_2, theta_3};
    }

private:
    double L1_, L2_, L3_, target_z_;
};



class IKnode : public rclcpp::Node
{
public:
    IKnode(RobotArm& arm) : Node("ik_node"), arm_(arm) {
        // create subscriber
        subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/coords", 10, std::bind(&IKnode::callback, this, std::placeholders::_1));



        // create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/angles", 10);
    }

private:
    // on receiving a message, calculate the joint angles and publish them
    void callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        std::vector<double> target_pos = {msg->x, msg->y};
        auto angles = arm_.inverse_kinematics(target_pos);

        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.name = {"base_rotation", "shoulder", "elbow", "wrist"};
        joint_state_msg.position = {angles[0], angles[1], angles[2], angles[3]};

        publisher_->publish(joint_state_msg);
    }



    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    RobotArm& arm_;
};





double positive_deg(double angle) {
    double deg = angle * 180.0 / M_PI;
    // same logic as %
    return std::fmod(deg, 360.0);
}

int main(int argc, char *argv[]) {
    // pass in L1, L2, L3, and target z height
    RobotArm arm_init(240.0, 290.0, 150.0, 53.1);


    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKnode>(arm_init);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}