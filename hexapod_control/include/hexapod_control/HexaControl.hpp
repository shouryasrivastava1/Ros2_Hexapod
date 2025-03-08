#ifndef hexapod_control__HexaControl_HPP_
#define hexapod_control__HexaControl_HPP_

#include "rclcpp/rclcpp.hpp"
#include "hexapod_msgs/msg/leg_info.hpp"
#include "hexapod_msgs/msg/leg_state.hpp"
#include "hexapod_kinematics/HexaKinematics.hpp"
#include <vector>

using namespace std;
using namespace hexapod_msgs::msg;
using namespace HexaKinematics;

namespace HexaControl
{
class ControlClass : public rclcpp::Node
{
    public:
        ControlClass() : Node("HexaController")
        {
            LegState = std::make_shared<hexapod_msgs::msg::LegState>();
            LegState->state.resize(6);
            RCLCPP_INFO(this->get_logger(), " resize done");
            for(int i = 0; i < 6; i++)
            {
                Leg.emplace_back(i, LegState->state[i], 0.0);
                RCLCPP_INFO(this->get_logger(), " looping");
            
            }

            RCLCPP_INFO(this->get_logger(), " legs done");

            pub_ = this->create_publisher<hexapod_msgs::msg::LegState>("/Hexapod/Legstate", 10);
            RCLCPP_INFO(this->get_logger(), " pub done");

            loc.resize(6);

            loc[LEG_0] = geometry_msgs::msg::Point();
            loc[LEG_1] = geometry_msgs::msg::Point();
            loc[LEG_2] = geometry_msgs::msg::Point();
            loc[LEG_3] = geometry_msgs::msg::Point();
            loc[LEG_4] = geometry_msgs::msg::Point();
            loc[LEG_5] = geometry_msgs::msg::Point();

            loc[LEG_0].x = -80;
            loc[LEG_0].y = 75;
            loc[LEG_0].z = 60;

            loc[LEG_1].x = 80;
            loc[LEG_1].y = 75;
            loc[LEG_1].z = 60;

            loc[LEG_2].x = 0;
            loc[LEG_2].y = 100;
            loc[LEG_2].z = 60;
            
            loc[LEG_3].x = 0;
            loc[LEG_3].y = 100;
            loc[LEG_3].z = 60;


            loc[LEG_4].x = 80;
            loc[LEG_4].y = 75;
            loc[LEG_4].z = 60;
            
            loc[LEG_5].x = -80;
            loc[LEG_5].y = 75;
            loc[LEG_5].z = 60;

            
            for(int i = 0; i < 6; i++) 
            {
                if(i == 0) Leg[i].PerformKinematics(loc[LEG_0]);
                else if(i == 1) Leg[i].PerformKinematics(loc[LEG_1]);
                else if(i == 2 ) Leg[i].PerformKinematics(loc[LEG_2]);
                else if(i == 3 ) Leg[i].PerformKinematics(loc[LEG_3]);
                else if(i == 4) Leg[i].PerformKinematics(loc[LEG_4]);
                else if(i == 5) Leg[i].PerformKinematics(loc[LEG_5]);
            }
            pub_->publish(*LegState);
            // RCLCPP_INFO(this->get_logger(), "X: %i | Y: %i | Z: %i", Leg[0].getCurrentPosition(POS_X),Leg[0].getCurrentPosition(POS_Y),Leg[0].getCurrentPosition(POS_Z));
            RCLCPP_INFO(this->get_logger(), "0: X: %i | Y: %i | Z: %i ", Leg[0].getCurrentPosition(POS_X),Leg[0].getCurrentPosition(POS_Y), Leg[0].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "1: X: %i | Y: %i | Z: %i ", Leg[1].getCurrentPosition(POS_X),Leg[1].getCurrentPosition(POS_Y), Leg[1].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "2: X: %i | Y: %i | Z: %i ", Leg[2].getCurrentPosition(POS_X),Leg[2].getCurrentPosition(POS_Y), Leg[2].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "3: X: %i | Y: %i | Z: %i ", Leg[3].getCurrentPosition(POS_X),Leg[3].getCurrentPosition(POS_Y), Leg[3].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "4: X: %i | Y: %i | Z: %i ", Leg[4].getCurrentPosition(POS_X),Leg[4].getCurrentPosition(POS_Y), Leg[4].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "5: X: %i | Y: %i | Z: %i ", Leg[5].getCurrentPosition(POS_X),Leg[5].getCurrentPosition(POS_Y), Leg[5].getCurrentPosition(POS_Z) );
            rclcpp::sleep_for(4s);
            for(int i = 0; i < 12; i++)
            {
                walkForward();
            }
            RCLCPP_INFO(this->get_logger(), "0: X: %i | Y: %i | Z: %i ", Leg[0].getCurrentPosition(POS_X),Leg[0].getCurrentPosition(POS_Y), Leg[0].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "1: X: %i | Y: %i | Z: %i ", Leg[1].getCurrentPosition(POS_X),Leg[1].getCurrentPosition(POS_Y), Leg[1].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "2: X: %i | Y: %i | Z: %i ", Leg[2].getCurrentPosition(POS_X),Leg[2].getCurrentPosition(POS_Y), Leg[2].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "3: X: %i | Y: %i | Z: %i ", Leg[3].getCurrentPosition(POS_X),Leg[3].getCurrentPosition(POS_Y), Leg[3].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "4: X: %i | Y: %i | Z: %i ", Leg[4].getCurrentPosition(POS_X),Leg[4].getCurrentPosition(POS_Y), Leg[4].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "5: X: %i | Y: %i | Z: %i ", Leg[5].getCurrentPosition(POS_X),Leg[5].getCurrentPosition(POS_Y), Leg[5].getCurrentPosition(POS_Z) );
            rclcpp::sleep_for(1s);
            ResetLegs();
            rclcpp::sleep_for(1s);
            for(int i = 0; i < 13; i++)
            {
                walkForward();
            }
            RCLCPP_INFO(this->get_logger(), "0: X: %i | Y: %i | Z: %i ", Leg[0].getCurrentPosition(POS_X),Leg[0].getCurrentPosition(POS_Y), Leg[0].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "1: X: %i | Y: %i | Z: %i ", Leg[1].getCurrentPosition(POS_X),Leg[1].getCurrentPosition(POS_Y), Leg[1].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "2: X: %i | Y: %i | Z: %i ", Leg[2].getCurrentPosition(POS_X),Leg[2].getCurrentPosition(POS_Y), Leg[2].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "3: X: %i | Y: %i | Z: %i ", Leg[3].getCurrentPosition(POS_X),Leg[3].getCurrentPosition(POS_Y), Leg[3].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "4: X: %i | Y: %i | Z: %i ", Leg[4].getCurrentPosition(POS_X),Leg[4].getCurrentPosition(POS_Y), Leg[4].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "5: X: %i | Y: %i | Z: %i ", Leg[5].getCurrentPosition(POS_X),Leg[5].getCurrentPosition(POS_Y), Leg[5].getCurrentPosition(POS_Z) );
            rclcpp::sleep_for(1s);
            ResetLegs();
            timer = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&ControlClass::callback, this));
        }

    private:
        vector<Kinematics> Leg;
        hexapod_msgs::msg::LegState::SharedPtr LegState;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<hexapod_msgs::msg::LegState>::SharedPtr pub_;
        vector<geometry_msgs::msg::Point> loc;
        bool Firststep = true;
        int i = 0;
        int phase = 0;

        int LEG_0 = 0;
        int LEG_1 = 1;
        int LEG_2 = 2;
        int LEG_3 = 3;
        int LEG_4 = 4;
        int LEG_5 = 5;

        void callback()
        {
            int a[5] = {40, 45, 50, 55, 60};
            // RCLCPP_INFO(this->get_logger(), "X: %f | Y: %f | Z: %f", loc.x, loc.y, loc.z);
            
            // for(int i = 0; i < 6; i++) 
            // {
            //     if(i == 0) Leg[i].PerformKinematics(loc[LEG_0]);
            //     else if(i == 1) Leg[i].PerformKinematics(loc[LEG_1]);
            //     else if(i == 2 ) Leg[i].PerformKinematics(loc[LEG_2]);
            //     else if(i == 3 ) Leg[i].PerformKinematics(loc[LEG_3]);
            //     else if(i == 4) Leg[i].PerformKinematics(loc[LEG_4]);
            //     else if(i == 5) Leg[i].PerformKinematics(loc[LEG_5]);
            // }
            // pub_->publish(*LegState) ;
        //     pub_->publish(*LegState);
            // walkForward();
            // RCLCPP_INFO(this->get_logger(), "Published");
            // i++;
            // if(i >= 5) i = 0;
            // pub_->publish(*LegState);11111


            // idk why there ig i willn nned it??
            // auto pos = geometry_msgs::msg::Point();
            // pos = loc;

            // pos.x = 55;

            // for(float t = 0; t < 1; t+=0.1)
            // {
            //     auto pos1 = geometry_msgs::msg::Point();
            //     pos1.y = loc.y;
            //     pos1.z = loc.z;
            //     pos1.x = lerp(loc.x, pos.x, t);
            //     cout << pos1.x << " "<<  t << endl;
            //     Leg[2].PerformKinematics(pos1);
            //     pub_->publish(*LegState);
            // }
                // rclcpp::sleep_for(100ms);
            

        }

        void walkForward()
        {
            int distance;
            int radius;

            if(Firststep)
            {
                distance = 20;
                radius = 25;
            }
            else
            {
                distance = 40;
                radius = 35;
            }

            Firststep = false;   
            for(double t = 0; t <= 1; t+=0.1)
            {
                if(phase == 0)
                {
                    Leg[0].MoveTo(POS_X, distance, 0, t);
                    RCLCPP_INFO(this->get_logger(), "phase: %i", phase);
                    RCLCPP_INFO(this->get_logger(), "t: %f", t);
                    RCLCPP_INFO(this->get_logger(), "Forward loop");

                    Leg[1].MoveTo(POS_X, distance, radius, t);

                    Leg[2].MoveTo(POS_X, -distance, radius, t);

                    Leg[3].MoveTo(POS_X, -distance, 0, t);

                    Leg[4].MoveTo(POS_X, distance, 0, t);

                    Leg[5].MoveTo(POS_X, distance, radius, t);
                    
                    pub_->publish(*LegState);
                    rclcpp::sleep_for(50ms);
                    RCLCPP_INFO(this->get_logger(), "X: %i | Y: %i | Z: %i", Leg[0].getCurrentPosition(POS_X),Leg[0].getCurrentPosition(POS_Y),Leg[0].getCurrentPosition(POS_Z));
                }

                else if(phase == 1)
                {
                    RCLCPP_INFO(this->get_logger(), "backward loop");
                    RCLCPP_INFO(this->get_logger(), "t: %f", t);
                    Leg[0].MoveTo(POS_X, -distance, radius, t);

                    Leg[1].MoveTo(POS_X, -distance, 0, t);

                    Leg[2].MoveTo(POS_X, distance, 0, t);

                    Leg[3].MoveTo(POS_X, distance, radius, t);

                    Leg[4].MoveTo(POS_X, -distance, radius, t);

                    Leg[5].MoveTo(POS_X, -distance, 0, t);
                    
                    pub_->publish(*LegState);
                    rclcpp::sleep_for(50ms);
                    RCLCPP_INFO(this->get_logger(), "X: %i | Y: %i | Z: %i", Leg[0].getCurrentPosition(POS_X),Leg[0].getCurrentPosition(POS_Y),Leg[0].getCurrentPosition(POS_Z));
                }
            }
            phase++;
            if(phase >= 2) phase = 0;
            RCLCPP_INFO(this->get_logger(), "phase: %i", phase);
            // rclcpp::sleep_for(100ms);
            // RCLCPP_INFO(this->get_logger(), "back loop");
            // for(double t = 0; t < 1; t+=0.1)
            // {
            //     Leg[0].MoveTo(POS_X, -distance, radius, t);
            //     RCLCPP_INFO(this->get_logger(), "t: %f", t);

            //     Leg[1].MoveTo(POS_X, -distance, 0, t);

            //     Leg[2].MoveTo(POS_X, distance, 0, t);

            //     Leg[3].MoveTo(POS_X, distance, radius, t);

            //     Leg[4].MoveTo(POS_X, -distance, radius, t);

            //     Leg[5].MoveTo(POS_X, -distance, 0, t);
                
            //     pub_->publish(*LegState);
            //     rclcpp::sleep_for(50ms);
            //     RCLCPP_INFO(this->get_logger(), "X: %i | Y: %i | Z: %i", Leg[0].getCurrentPosition(POS_X),Leg[0].getCurrentPosition(POS_Y),Leg[0].getCurrentPosition(POS_Z));
            // } 
            // pub_->publish(*LegState);
        }

        void ResetLegs()
        {
            vector<int> distance;
            distance.resize(6);
            int radius = 20;
           
            if(Leg[0].getCurrentPosition(POS_X) != loc[0].x)
            {
                distance[0] = loc[0].x - Leg[0].getCurrentPosition(POS_X) + 4;
                distance[3] = loc[3].x - Leg[3].getCurrentPosition(POS_X) - 4;
                distance[4] = loc[4].x - Leg[4].getCurrentPosition(POS_X) + 5;
                for(double t = 0; t <= 1; t+=0.1)
                {
                    RCLCPP_INFO(this->get_logger(), "t: %f", t);
                    Leg[0].MoveTo(POS_X, distance[0], radius, t);

                    Leg[3].MoveTo(POS_X, distance[3], radius, t);

                    Leg[4].MoveTo(POS_X, distance[4], radius, t);

                    pub_->publish(*LegState);
                    rclcpp::sleep_for(50ms);
                }
            }
            
      
            if(Leg[1].getCurrentPosition(POS_X) != loc[1].x)
            {
                distance[1] = loc[1].x - Leg[1].getCurrentPosition(POS_X) + 5;
                distance[2] = loc[2].x - Leg[2].getCurrentPosition(POS_X) - 5;
                distance[5] = loc[5].x - Leg[5].getCurrentPosition(POS_X) + 4;
                for(double t = 0; t <= 1; t+=0.1)
                {
                    RCLCPP_INFO(this->get_logger(), "t: %f", t);
                    Leg[1].MoveTo(POS_X, distance[1], radius, t);

                    Leg[2].MoveTo(POS_X, distance[2], radius, t);

                    Leg[5].MoveTo(POS_X, distance[5], radius, t);

                    pub_->publish(*LegState);
                    rclcpp::sleep_for(50ms);
                }
            }
                
            phase = 0;
            Firststep = true;
            RCLCPP_INFO(this->get_logger(), "Stoped legs");
            RCLCPP_INFO(this->get_logger(), "0: X: %i | Y: %i | Z: %i ", Leg[0].getCurrentPosition(POS_X),Leg[0].getCurrentPosition(POS_Y), Leg[0].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "1: X: %i | Y: %i | Z: %i ", Leg[1].getCurrentPosition(POS_X),Leg[1].getCurrentPosition(POS_Y), Leg[1].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "2: X: %i | Y: %i | Z: %i ", Leg[2].getCurrentPosition(POS_X),Leg[2].getCurrentPosition(POS_Y), Leg[2].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "3: X: %i | Y: %i | Z: %i ", Leg[3].getCurrentPosition(POS_X),Leg[3].getCurrentPosition(POS_Y), Leg[3].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "4: X: %i | Y: %i | Z: %i ", Leg[4].getCurrentPosition(POS_X),Leg[4].getCurrentPosition(POS_Y), Leg[4].getCurrentPosition(POS_Z) );
            RCLCPP_INFO(this->get_logger(), "5: X: %i | Y: %i | Z: %i ", Leg[5].getCurrentPosition(POS_X),Leg[5].getCurrentPosition(POS_Y), Leg[5].getCurrentPosition(POS_Z) );
        }


};
}  // namespace HexaControl
#endif  // hexapod_control__HexaControl_HPP_
