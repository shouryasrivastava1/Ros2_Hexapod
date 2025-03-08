#include "hexapod_kinematics/HexaKinematics.hpp"

using namespace HexaKinematics;

Kinematics::Kinematics(int leg_id, LegInfo &state, double coxa_offset) : LegState(&state)
{
    this->LegState->leg_id = leg_id;
    this->LegState->coxa_offset= coxa_offset;
    

}


void Kinematics::PerformKinematics(geometry_msgs::msg::Point loc, int update)
{

    if(!(abs(loc.x * loc.x + loc.y * loc.y ) <= 1e-6))
    {
        this->LegState->coxa_radians =   (atan2(loc.x , loc.y));
        if(this->LegState->leg_id == 0 ){ this->LegState->coxa_radians+= degrees2radians(50); this->LegState->coxa_radians-=0.0550195537507534;}
        else if(this->LegState->leg_id == 4) {this->LegState->coxa_radians-= degrees2radians(50); this->LegState->coxa_radians-=0.0550195537507534;}
        else if(this->LegState->leg_id == 1){ this->LegState->coxa_radians-= degrees2radians(50); this->LegState->coxa_radians+= 0.0550195537507534;}
        else if(this->LegState->leg_id == 5){this->LegState->coxa_radians+= degrees2radians(50); this->LegState->coxa_radians+= 0.0550195537507534;}

    }

    float L = sqrt(pow(loc.x, 2) + pow(loc.y, 2)) - COXA_LEN;

    float H = sqrt(pow(loc.z, 2) + pow(L, 2));

    float theta_2 = acos((pow(FEMUR_LEN, 2) + pow(H, 2) - pow(TIBIA_LEN, 2)) / ( 2 * FEMUR_LEN * H));

    float theta_1 = atan2(loc.z, L);

    this->LegState->femur_radians = -(theta_2 - theta_1);

    //tibia ka part
    this->LegState->tibia_radians = (degrees2radians(90) - acos((pow(FEMUR_LEN, 2) + pow(TIBIA_LEN, 2) - pow(H, 2)) / (2 * FEMUR_LEN * TIBIA_LEN)));
    

    
    this->LegState->tibia_radians += degrees2radians(TIBIA_OFFSET); // offset = 2.5 deg
    

    // TODO: add the offsets for servos in degrees
    //this is the conversion part randians to deg
    this->LegState->coxa = radians2degrees(this->LegState->coxa_radians + degrees2radians(this->COXA_SERVO_OFFSET));

    this->LegState->femur =   (radians2degrees(this->LegState->femur_radians) + 90);

    this->LegState->tibia = 180 - (radians2degrees(this->LegState->tibia_radians) + 90);

    if(update) CurrentPositon = loc;
    AcctualCurrentPositon = loc;
    // cout << "X: " << loc.x << "y: " << loc.y << " Z: " << loc.z << endl;

    
}

void Kinematics::Populate() 
{

    this->LegState->coxa = rand() % 180;
    this->LegState->femur = rand() % 180;
    this->LegState->tibia = rand() % 180;
}

double Kinematics::degrees2radians(double deg)
{
    return deg * (M_PI/180);
}

double Kinematics::radians2degrees(double rad)
{
    return rad * (180 / M_PI);
}

int Kinematics::getCurrentPosition(int type)
{
    return (type == POS_X) ? AcctualCurrentPositon.x : (type == POS_Y) ? AcctualCurrentPositon.y : (type == POS_Z) ? AcctualCurrentPositon.z : -1;
}

double Kinematics::getAngle(int component, int type)
{
    if(component == COMPONENT_COXA)
    {
        if(type == ANGLE_DEG) return this->LegState->coxa;
        else if(type == ANGLE_RAD) return this->LegState->coxa_radians;
        else return -1;
    }
    else if(component == COMPONENT_FEMUR)
    {
        if(type == ANGLE_DEG) return this->LegState->femur;
        else if(type == ANGLE_RAD) return this->LegState->femur_radians;
        else return -1;
    }
    else if(component == COMPONENT_TIBIA)
    {
        if(type == ANGLE_DEG) return this->LegState->tibia;
        else if(type == ANGLE_RAD) return this->LegState->tibia_radians;
        else return -1;
    }
    return -1;

}

void Kinematics::MoveTo(int type, int distance, int radius, double t)
{
    int lerpType = (radius == 0) ? INTERPOLATION_LINEAR : INTERPOLATION_QUAD;

    auto position = geometry_msgs::msg::Point();

    position = CurrentPositon;

    if(type == POS_X) position.x += distance;
    else if(type == POS_Y) position.y += distance;
    else if(type == POS_Z) return;
    // cout << position.y << endl;

    if(lerpType == INTERPOLATION_LINEAR) lerp(CurrentPositon, position, t);
    else if(lerpType == INTERPOLATION_QUAD)
    {
        auto control = geometry_msgs::msg::Point();
        control = CurrentPositon;

        if(type == POS_X) control.x += (position.x - CurrentPositon.x ) / 2;
        else if(type == POS_Y) control.y = (position.y - CurrentPositon.y ) / 2;
        else if(type == POS_Z) return;
        
        control.z -= radius;
        // cout << "X: " << control.x << "y: " << control.y << " Z: " << control.z << endl;

        qerp(CurrentPositon, control, position, t, type);
    }
    if(t >= 1.0 - 1e-6)
    {
        CurrentPositon = AcctualCurrentPositon;
        cout << "position saved \n";
        cout << this->LegState->leg_id << " "<<  CurrentPositon.x << endl;
    }
    
    
}

geometry_msgs::msg::Point Kinematics::lerp(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end, double t, int type)
{
    auto position = geometry_msgs::msg::Point();
    position.x = start.x + t * (end.x - start.x); // get lerped positons for all
    position.y = start.y + t * (end.y - start.y);
    position.z = start.z + t * (end.z - start.z);

    if(type != FUNCTION_RETURN) PerformKinematics(position, (t >= 1.0) ? 1 : UPDATE_NO); // give it to the kinematics model to set the angles :)
    else return position;
    return geometry_msgs::msg::Point();
}

void Kinematics::qerp(geometry_msgs::msg::Point start, geometry_msgs::msg::Point control, geometry_msgs::msg::Point end, double t, int type)
{
    auto result = geometry_msgs::msg::Point();
    auto start2control = geometry_msgs::msg::Point();
    auto control2end = geometry_msgs::msg::Point();
    auto followpoint = geometry_msgs::msg::Point();

    start2control = lerp(start, control, t, FUNCTION_RETURN);
    control2end = lerp(control, end, t, FUNCTION_RETURN);
    followpoint = lerp(start2control, control2end, t, FUNCTION_RETURN);

    if(type == POS_X)
    {
        result.y = start.y + t * (end.y - start.y);
    }
    else if(type == POS_Y)
    {
        result.x = start.x + t * (end.x - start.x);
    }
    else return;
    // cout << "X: " << control.x << "y: " << control.y << " Z: " << control.z << endl;
    PerformKinematics(followpoint, (t >= 1.0) ? 1 : UPDATE_NO);
}