#ifndef hexapod_sim__ControlTopicConverter_HPP_
#define hexapod_sim__ControlTopicConverter_HPP_

#include "rclcpp/rclcpp.hpp"
#include "hexapod_msgs/msg/leg_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
namespace TopicConverter
{
class ConverterClass : public rclcpp::Node
{
    public:
        ConverterClass();


    private:
        rclcpp::Subscription<hexapod_msgs::msg::LegState>::SharedPtr hexa_sub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_pub;

        void callabck(const hexapod_msgs::msg::LegState::SharedPtr state);

};
}  // namespace TopicConverter
#endif  // hexapod_sim__ControlTopicConverter_HPP_
