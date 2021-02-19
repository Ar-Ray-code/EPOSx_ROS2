#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>

class twist2wheels: public rclcpp::Node
{
private:
    void sub_twist_thread(const geometry_msgs::msg::Twist::SharedPtr msg);

public:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor0;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor1;

    twist2wheels(const std::string &name_space);
};

twist2wheels::twist2wheels(const std::string &name_space):Node(name_space)
{
    sub_twist = this->create_subscription<geometry_msgs::msg::Twist>
		("cmd_vel", 1, std::bind(&twist2wheels::sub_twist_thread, this, std::placeholders::_1));
    motor0 = this->create_publisher<std_msgs::msg::Int32>("motor0",1);
    motor1 = this->create_publisher<std_msgs::msg::Int32>("motor1",1);
}

void twist2wheels::sub_twist_thread(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std_msgs::msg::Int32 spd0_data, spd1_data;

    // x
    spd0_data.data += int(msg->linear.x*100);
    spd1_data.data -= int(msg->linear.x*100);
    // y
    spd0_data.data += int(msg->angular.z*100);
    spd1_data.data += int(msg->angular.z*100);

    motor0->publish(spd0_data);
    motor1->publish(spd1_data);
    
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<twist2wheels>("twist2wheels");
	
    rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
