#include "cmath"
#include "hexapod_msgs/msg/leg_info.hpp"
#include "hexapod_msgs/msg/leg_state.hpp"
#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include <iostream>

using namespace std;
using namespace hexapod_msgs::msg;
using namespace geometry_msgs::msg;

namespace HexaKinematics
{

    #define POS_X 0
    #define POS_Y 1
    #define POS_Z 2

    #define INTERPOLATION_LINEAR 4
    #define INTERPOLATION_QUAD 5

    #define ANGLE_DEG 6
    #define ANGLE_RAD 7

    #define COMPONENT_COXA 8
    #define COMPONENT_FEMUR 9
    #define COMPONENT_TIBIA 10

    #define FUNCTION_RETURN 11

    #define UPDATE_NO 0

    class Kinematics
    {
        public:
            Kinematics(int leg_id, LegInfo &state, double coxa_offset = 0.0);

            void PerformKinematics(geometry_msgs::msg::Point loc, int update = 1);
            void Populate();
            int getCurrentPosition(int type);
            double getAngle(int component, int type);
            void MoveTo(int type, int distance, int radius = 0, double t = 0.0);
        private:
            LegInfo::SharedPtr LegState;
            geometry_msgs::msg::Point CurrentPositon;
            geometry_msgs::msg::Point AcctualCurrentPositon;

            double COXA_LEN = 43;
            double FEMUR_LEN = 60;
            double TIBIA_LEN = 109;

            double TIBIA_OFFSET = 2.5;
            double TIBIA_SERVO_OFFSET = 180.0;
            double FEMUR_SERVO_OFFSET = 90.0;
            double COXA_SERVO_OFFSET = 90.0;

            bool storeCurrentPositon = true;

            double degrees2radians(double deg);
            double radians2degrees(double rad);

            geometry_msgs::msg::Point lerp(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end, double t, int type = 0);
            void qerp(geometry_msgs::msg::Point start, geometry_msgs::msg::Point control, geometry_msgs::msg::Point end, double t, int type = POS_X);

        
    };
}