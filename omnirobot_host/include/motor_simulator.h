#include "ros/ros.h"
#include "omnirobot_msgs/MotorSpeed.h"

/*
MotorSimulator offers same api as Adafruit MotorHAT,
Thus it can be used for debug without a real robot.
This simulator can be used with encoder_simulator, offering a complete loopback.
*/
//#define SIMULATE_ENCODER_LEVEL

class MotorSimulator{
private:
    int _direction;
    int _id;
    ros::Publisher _motor_command_pub;
    float _speed;
    ros::NodeHandle* _n_ptr;
public:
    MotorSimulator(int id, ros::NodeHandle *n_ptr){
        _id = id;
        _direction = 0; //forward
        _n_ptr = n_ptr;
        switch(id) {
            case 1: {
                _motor_command_pub = n_ptr->advertise<omnirobot_msgs::MotorSpeed>("/wheel_left_joint_controller", 1000);
                break;
            }
            case 2: {
                _motor_command_pub = n_ptr->advertise<omnirobot_msgs::MotorSpeed>("/wheel_front_joint_controller", 1000);
                break;
            }
            case 3: {
                _motor_command_pub = n_ptr->advertise<omnirobot_msgs::MotorSpeed>("/wheel_right_joint_controller", 1000);
                break;
            }
        }
    };
    void run(int direction){
        /*
        enum Direction {
            FORWARD,
            BACKWARD,
            BRAKE,
            RELEASE
        };
        */
       _direction = direction;
    }
    void setSpeed(float speed){
        switch(_direction){
            case 0:{
                //forward
                _speed = speed;
                //ROS_INFO("forward, speed = %d", _speed);
                break;
            }
            case 1:{
                //backward
                _speed = - speed;
                //ROS_INFO("backward, speed = %d", _speed);
                break;
            }
            default:{
                _speed = 0;
            }
        }
        updateCommand();
    }
    void updateCommand(){
        omnirobot_msgs::MotorSpeed msg;
        msg.id = _id;
        msg.speed = _speed;
        _motor_command_pub.publish(msg);
    }
};