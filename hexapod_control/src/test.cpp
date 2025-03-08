#include "rclcpp/rclcpp.hpp"
#include "hexapod_msgs/msg/leg_state.hpp"
#include "hexapod_control/HexaControl.hpp"
namespace test_space
{
class test_class : public rclcpp::Node
{
public:
    test_class() : Node("test_hexapod_node")
    {
        pub_ = this->create_publisher<hexapod_msgs::msg::LegState>("Test/Hexastate", 10);
        timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&test_class::callback, this));
        RCLCPP_INFO(this->get_logger(),"Stated timer");
    }

private:
    rclcpp::Publisher<hexapod_msgs::msg::LegState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer;
    void callback()
    {
        hexapod_msgs::msg::LegState LegState;
        LegState.state.resize(1);
        LegState.state[0].leg_id = 0;
        LegState.state[0].coxa = 1.57;
        LegState.state[0].femur = 0.32;
        LegState.state[0].tibia = 3.14;
        LegState.state[0].coxa_offset = 2.33;

        pub_->publish(LegState);
        RCLCPP_INFO(this->get_logger(), "Published");
    }

};
}  


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexaControl::ControlClass>());
    return 0;
}

