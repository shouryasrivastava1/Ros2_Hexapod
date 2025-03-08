#include "hexapod_Sim/ControlTopicConverter.hpp"

sensor_msgs::msg::JointState msg;

TopicConverter::ConverterClass::ConverterClass() : Node("HexapodControlTopic2SimTopicConverter") 
{
    sim_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    hexa_sub = this->create_subscription<hexapod_msgs::msg::LegState>("/Hexapod/Legstate", 10, std::bind(&ConverterClass::callabck, this, std::placeholders::_1));
    msg = sensor_msgs::msg::JointState();
    msg.name.resize(18);
    msg.position.resize(18);
    int leg = 0;
    for(int i = 0; i < 18; i+=3)
    {
        msg.name[i] = "coxa_joint" + std::to_string(leg);
        msg.name[i+1] = "femur_joint" + std::to_string(leg);
        msg.name[i+2] = "tibia_joint" + std::to_string(leg);
        leg++;

    }

}

void TopicConverter::ConverterClass::callabck(const hexapod_msgs::msg::LegState::SharedPtr state)
{

    msg.header.stamp = rclcpp::Clock().now();


    
    // msg.position = {0.86, 0.78, 1.57};
    int leg = 0;
    for(int i = 0; i < 18; i+=3)
    {
        msg.position[i] = state->state[leg].coxa_radians;
        msg.position[i+1] = state->state[leg].femur_radians;
        msg.position[i+2] = state->state[leg].tibia_radians;
        leg++;
    }



    sim_pub->publish(msg);
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicConverter::ConverterClass>());
    return 0;
}
