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
#include "omniwheel_robot/motor_controller.h"


MotorController *motor1_ptr, *motor2_ptr, *motor3_ptr;
float wheel_speed_command_left = 0.0; 
float wheel_speed_command_right = 0.0; 
float wheel_speed_command_front = 0.0; 
ros::Time last_command_update_time;

void stopSignalCallback(const std_msgs::Bool::ConstPtr& msg){
    motor1_ptr->stop();
    motor1_ptr->stop();
    motor1_ptr->stop();
}
void resetSignalCallback(const std_msgs::Bool::ConstPtr& msg){
    motor1_ptr->reset();
    motor1_ptr->reset();
    motor1_ptr->reset();
}

void wheelSpeedCommandCallback(const omnirobot_msgs::WheelSpeeds::ConstPtr& msg){
    wheel_speed_command_left = msg->left;
    wheel_speed_command_front = msg->front;
    wheel_speed_command_right = msg->right;
    last_command_update_time = ros::Time::now();
}

/***************************************************/
/*                                       MAIN                                     */
/***************************************************/
int main(int argc,char ** argv)
{
    /* ROS init */
    ros::init(argc, argv, "wheel_speed_controller");
    ros::NodeHandle n;
   last_command_update_time = ros::Time::now();
    /* Calls the function ctrl_c_handler on interrupt signal (such as CTRL+C) */
    signal(SIGINT, motor_ctrl_c_handler);
    /* Geometric parameters */
    float L = 0.120; // m
    float R = 0.024; // m

    /* max angular speed of the motors, in rad/s
    * /!\ peek speed has been observed at 17 rad/s (no torque), but burned the H-bridge after few seconds.
    * Please never exceed 10 rad/s */
    float max_angular_speed = 8.0 ; //  rad/s
    float max_wheel_speed = max_angular_speed * R; //m/s

    /* Create a controller for each motor
    *  MotorController::MotorController(int num, float max_speed, float radius, float length, ros::NodeHandle *node_handle)
    *  Where num is the motor ID, max_speed the max angular speed of a motor, radius the radius of the wheel, length the
    *   length between the wheel and the center of the robot, node_handle the NodeHandle previsouly launched.
    *  */

    MotorController Motor1(1, max_angular_speed, R,  L, &n);
    MotorController Motor2(2, max_angular_speed, R,  L, &n);
    MotorController Motor3(3, max_angular_speed, R,  L, &n);
    motor1_ptr = &Motor1;
    motor2_ptr = &Motor2;
    motor3_ptr = &Motor3;

    ros::Subscriber reset_signal_sub = n.subscribe("/reset", 100, &resetSignalCallback);
    ros::Subscriber stop_signal_sub = n.subscribe("/stop", 100, &stopSignalCallback);
    ros::Subscriber wheel_speed_command_sub = n.subscribe("/wheel_speeds_commands", 10, &wheelSpeedCommandCallback);
    /* As we use a PID controller, it is better to set a frequency for calling ...*/
    ros::Rate loop_rate(50); // 50 Hz loop

    while (ros::ok())
    {
        if ((ros::Time::now() - last_command_update_time).toSec() > 1) // 1s time out 
        {
            // if no new command comes within 1 second, then brake
            wheel_speed_command_left = 0.0;
            wheel_speed_command_right = 0.0;
            wheel_speed_command_front = 0.0;
        }
        Motor1.updateWheelSpeedCommand(wheel_speed_command_left);
        Motor2.updateWheelSpeedCommand(wheel_speed_command_front);
        Motor3.updateWheelSpeedCommand(wheel_speed_command_right);

        ros::spinOnce();
        loop_rate.sleep();
    }

    Motor1.free();
    Motor2.free();
    Motor3.free();

    // Return value
    return 0;
}
