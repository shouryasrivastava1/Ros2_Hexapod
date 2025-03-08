#include "hexapod_Sim/ControlTopicConverter.hpp"

TopicConverter::ConverterClass::ConverterClass() : Node("HexapodControlTopic2SimTopicConverter") 
{
    sim_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    hexa_sub = this->create_subscription<hexapod_msgs::msg::LegState>("/Hexapod/Legstate", 10, std::bind(&ConverterClass::callabck, this, std::placeholders::_1));
}

void TopicConverter::ConverterClass::callabck(const hexapod_msgs::msg::LegState::SharedPtr state)
{
    auto msg = sensor_msgs::msg::JointState();

    msg.header.stamp = rclcpp::Clock().now();

    msg.name = {"coxa_joint", "femur_joint", "tibia_joint"};

    msg.position = {state->state[0].coxa_radians, state->state[0].femur_radians, state->state[0].tibia_radians};
    
    // msg.position = {0.86, 0.78, 1.57};


    sim_pub->publish(msg);
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicConverter::ConverterClass>());
    return 0;
}
