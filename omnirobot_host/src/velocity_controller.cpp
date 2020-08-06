#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "omnirobot_msgs/WheelSpeeds.h"
#include "stdio.h"
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <math.h> // cos and sin

//#define DEBUG
#define SIMULATE_BYPASS_ENCODER_LEVEL
float absMax(const float & x,const  float& y, const float& z){
    //return maximum fabs value of three input
  float max_value=0;
  float abs_x, abs_y, abs_z;
  abs_x = fabs(x);
  abs_y = fabs(y);
  abs_z = fabs(z);
  if(max_value < abs_x) max_value = abs_x;
  if(max_value < abs_y) max_value = abs_y;
  if(max_value < abs_z) max_value = abs_z;
  return max_value;
}


class VelocityController{
    /*
    This class encapsulates the whole robot's dynamic, the input is the robot relative velocity commands 
    this class will transform them into each wheel's speed command, and use motor_controller to control each wheel.
    */

private:
    /* Speed inputs */
    float _Vxr, _Vyr, _wr;
    // _Vxr : forward speed in robot's local frame (m/s)
    // _Vyr : left speed in robot's local frame (m/s)
    // _wr : rotation speed around z-axis in robot's local frame (rad/s)
    ros::Time _last_command_update_time;
    /* geometry */
    float _R;
    float _L;
    /* constraint */
    float _max_wheel_speed; //max wheel speed in m/s

    float _speed_command_left, _speed_command_front, _speed_command_right;


    ros::Publisher _wheel_speeds_command_pub;
    #ifdef SIMULATE_BYPASS_ENCODER_LEVEL
    ros::Publisher _wheel_speeds_simulator_pub;
    #endif
    float _stay_stop;
    ros::Subscriber _velocity_command_sub, _switch_sub;
    ros::NodeHandle _n;
    bool _enabled; //switch to enable or disable this controller
public:
    VelocityController(float wheel_radius, float length, float max_wheel_speed, ros::NodeHandle& n){
        _R = wheel_radius;
        _L = length;
        _n = n;

        _enabled = true;
        
        _Vxr = 0.0;
        _Vyr = 0.0;
        _wr = 0.0;

        _last_command_update_time = ros::Time::now();

        _max_wheel_speed = max_wheel_speed; // in m/s
        /*  max_wheel_speed is used to constaint maximum wheel speed of each moment.
            This is usefull to ensure the robot can always do the translation orientated of any angle, 
            because we should garrantee that no wheel's speed is saturated.
        */
        _stay_stop = false;
        
        _switch_sub =  _n.subscribe("/switch/velocity_controller", 1, &VelocityController::switchCallback, this);
        _velocity_command_sub = _n.subscribe("velocity_commands", 100, &VelocityController::velComCallback, this);
        _wheel_speeds_command_pub = _n.advertise<omnirobot_msgs::WheelSpeeds>("/wheel_speeds_commands", 100);
        #ifdef SIMULATE_BYPASS_ENCODER_LEVEL
        // send /wheel_speeds_command msg directly to /wheel_speeds, to simulate a perfect motor
        _wheel_speeds_simulator_pub = _n.advertise<omnirobot_msgs::WheelSpeeds>("/wheel_speeds", 100);
        #endif
    }

    void reset()
    {
        _Vxr = 0;
        _Vyr = 0;
        _wr = 0;
        _stay_stop = false;

    }

    void stop(){
        _stay_stop = true;
    }

    void switchCallback(const std_msgs::Bool::ConstPtr& msg){
        _enabled = msg->data;
	if(_enabled) ROS_INFO("velocity_controller is enabled");
	else ROS_INFO("velocity_controller is disabled");
    }
    /* Velocity input callback */
    void velComCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        _Vxr = msg->linear.x;
        _Vyr = msg->linear.y;
        _wr = msg->angular.z;
        _last_command_update_time = ros::Time::now();
	if(!_enabled) ROS_INFO("recieved new velocity command, however, velocity_controller is disabled");
    }

    void calculWheelSpeedsCommand(){
        if (_enabled){
            // time out 1s or in stay stop state
            if(_stay_stop || (ros::Time::now() -_last_command_update_time).toSec() > 1.0){
                _speed_command_left = 0;
                _speed_command_front = 0;
                _speed_command_right = 0;
            }
            else{
                _speed_command_left = _Vxr*sqrt(3)/2 + _Vyr/2 - _wr*_L; //(m/s) speed
                _speed_command_front = -_Vyr - _wr*_L;
                _speed_command_right = -_Vxr*sqrt(3)/2 + _Vyr/2 - _wr*_L;

                float curr_max_speed = absMax(_speed_command_left, _speed_command_front, _speed_command_right);
                if(curr_max_speed > _max_wheel_speed){
                    // normalize the speed
                    float ratio =  _max_wheel_speed / curr_max_speed;
                    _speed_command_left = _speed_command_left * ratio;
                    _speed_command_front = _speed_command_front * ratio;
                    _speed_command_right = _speed_command_right * ratio;
                }
            }
            #ifdef DEBUG
            ROS_INFO("DEBUG: _speed_command_left %f, _speed_command_front %f, _speed_command_right %f",_speed_command_left, _speed_command_front, _speed_command_right);
            #endif
        }

    }
    float getLeftWheelSpeedCommand(){
        return this->_speed_command_left;
    }
    float getFrontWheelSpeedCommand(){
        return this->_speed_command_front;
    }
    float getRightWheelSpeedCommand(){
        return this->_speed_command_right;
    }

    void pubWheelSpeedCommand(){
        if(_enabled){
            omnirobot_msgs::WheelSpeeds msg;
            msg.left = _speed_command_left; //linear speed
            msg.front = _speed_command_front;
            msg.right = _speed_command_right;
            _wheel_speeds_command_pub.publish(msg);

            #ifdef SIMULATE_BYPASS_ENCODER_LEVEL
            // send /wheel_speeds_command msg directly to /wheel_speeds, to simulate a perfect motor
            _wheel_speeds_simulator_pub.publish(msg);
            #endif
        }
    }
};




/***************************************************/
/*                                       MAIN                                     */
/***************************************************/
int main(int argc,char ** argv)
{
    /* ROS init */
    ros::init(argc, argv, "velocity_controller");
    ros::NodeHandle n;

    /* Geometric parameters */
    float L = 0.120; // m
    float R = 0.024; // m

    /* max angular speed of the motors, in rad/s
    * /!\ peek speed has been observed at 17 rad/s (no torque), but burned the H-bridge after few seconds.
    * Please never exceed 10 rad/s */
    float max_angular_speed = 8.0 ; //  rad/s
    float max_wheel_speed = max_angular_speed * R; //m/s

    VelocityController velocity_controller(R, L, max_wheel_speed, n);

    /* As we use a PID controller, it is better to set a frequency for calling ...*/
    ros::Rate loop_rate(50); // 50 Hz loop

    while (ros::ok())
    {
        /***************      Kinematics Relations **************/
        /*    Relative velocities :                                                      */
        /*    Vxr = sqrt(3)*R*(w1-w3)                                             */
        /*    Vyr = R*(w1+w3)/3 -2*R*w2/3                                  */
        /*    wr = -R(w1+w2+w3)/(3*L)                                          */
        /*                                                                                           */
        /*     Global velocities :                                                       */
        /*     Vx = Vxr*cos(heading) - Vyr*sin(heading)              */
        /*     Vy = Vxr*sin(heading) + Vyr*cos(heading)             */
        /******************************************************/
        velocity_controller.calculWheelSpeedsCommand();
        velocity_controller.pubWheelSpeedCommand();
        ros::spinOnce();
        loop_rate.sleep();
    }
    // Return value
    return 0;
}
