#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "omnirobot_msgs/WheelSpeeds.h"
#include "omnirobot_host/motor_simulator.h"
#include "stdio.h"
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <math.h> // cos and sin

#define ticks_to_rads (2*M_PI/158)
#define timeout (1.0) // sec
//#define DEBUG
#define PID_MAN_TUNE

//use real motor
/*************** Parameters & Variables ******************/
// Adafruit_MotorHAT hat;
/*
 *  xr is the front axis in robot frame, aligned with camera and DC Motor 2
 *  yr is left axis in robot frame when xr is forward, between DC Motor 2 and 1
 *
 *  R : wheel radius
 *  L : distance between a wheel and the center of the robot
 *  wr : angular speed of the robot
 *  wi : angular speed of the wheel i
 *
 *  w1 = Vxr * sqrt(3)*R/2  + Vyr /(2*R) - w*L/R
 *  w2 = - Vyr/R - w*L/R
 *  w3 = - Vxr * sqrt(3)*R/2  + Vyr /(2*R) - w*L/R
*/


/************* Controller class ***************************/

class MotorController
{
  /*
    this class encapsulates a physical motor, the input signal is wheel speed, then it uses PID to control the motor to closing this speed.
  */
public:
  MotorController(int num, float max_speed, float radius, float length, ros::NodeHandle *node_handle)
  {
    /* subscribe to the current wheel speed topic */
    _wheel_speed_sub = node_handle->subscribe("/wheel_speeds", 1000, &MotorController::callback, this);

    _current_wheel_speed = 0.0; //(m/s)
    _max_angular_speed = max_speed;
    _R = radius;
    _L = length;

    /* init PID
     * Tuned with heuristic Ziegler-Nichols method : unefficient. Encoder values are not precise enough to analyze
     * oscillations frequency. We prefer a muanually tuned PI control */
    _PID_last_time = ros::Time::now();
    _errSum = 0.0;
    _lastErr = 0.0;
    _windup = 100.0;
    _Kp = 0.4;
    _Ki = 12;
    _Kd = 0;
    //_Kp = 0.20;
    //_Ki = 4.00;
    //_Kd = 0.0;

    _stay_stop = false;
    /* Init motor */
    _motor_id = num;
    _pMotor = new MotorSimulator(num, node_handle);

    _pMotor->setSpeed(0);
    _pMotor->run(FORWARD);
    _pMotor->run(RELEASE);

#ifdef PID_MAN_TUNE
    /* Allow PID tuning by publishing new coeff */
    _debug_Kp_sub = node_handle->subscribe("/tune_pid/Kp", 1, &MotorController::debugKpCallback, this);
    _debug_Ki_sub = node_handle->subscribe("/tune_pid/Ki", 1, &MotorController::debugKiCallback, this);
    _debug_Kd_sub = node_handle->subscribe("/tune_pid/Kd", 1, &MotorController::debugKdCallback, this);
    _debug_windup_sub = node_handle->subscribe("/tune_pid/windup", 1, &MotorController::debugWindupCallback, this);

#endif

  }

#ifdef PID_MAN_TUNE
  void debugKpCallback(const std_msgs::Float64::ConstPtr& msg)
  {
     _Kp = msg->data;
    ROS_INFO("new _Kp : %lf", _Kp);
  }
  void debugKiCallback(const std_msgs::Float64::ConstPtr& msg)
  {
     _Ki = msg->data;
    ROS_INFO("new _Ki : %lf", _Ki);
  }
  void debugKdCallback(const std_msgs::Float64::ConstPtr& msg)
  {
     _Kd = msg->data;
    ROS_INFO("new _Kd : %lf", _Kd);
  }
  void debugWindupCallback(const std_msgs::Float64::ConstPtr & msg){
    _windup = msg->data;
    ROS_INFO("new _windup : %lf", _windup);
  }
#endif

  void callback(const omnirobot_msgs::WheelSpeeds::ConstPtr& msg)
  {
    switch(_motor_id){
      case 1:
      {
        _current_wheel_speed = msg->left;
        break;
      }
      case 2:
      {
        _current_wheel_speed = msg->front;
        break;
      }
      case 3:
      {
        _current_wheel_speed = msg->right;
        break;
      }
    }
  }

/* Compute the angular speed according to the relative wheel velocity speed_command (m/s)
 *  0 means stop, 255 means max_speed */
  void updateWheelSpeedCommand(float speed_command)
  {
    if(_stay_stop){
      _pMotor->setSpeed(0);
      return;
    }
    float speed_input;
  
    // hard to simulate with pid, thus suppose motor controller is perfect, motor speed = motor speed command
    speed_input = speed_command/ _R;
    /* Check direction */
    if (speed_input < 0)
    {
      _pMotor->run(BACKWARD);
      speed_input = -speed_input; // Just keep abs value
    }
    else _pMotor->run(FORWARD);

    if(speed_input > _max_angular_speed) //max linear speed
      speed_input = _max_angular_speed;
    _pMotor->setSpeed((speed_input*255/17.0));
    //ROS_INFO("id: %d, set speed: %f", _motor_id, speed_input);
  
  }


  /* Release the motor */
  void free()
  {
     _pMotor->run(RELEASE);
     delete _pMotor;
     ROS_INFO("Release Motor %d", _motor_id);
  }

  void stop(){
    _stay_stop = true;
  }
  void reset(){
    _errSum = 0;
    _stay_stop = false;
  }

private:
  bool _stay_stop; //emergency stop

  /* Subscriber to get encoder value */
  ros::Subscriber _wheel_speed_sub;
  /* Timer for encoder watchdog */
  ros::Timer _timer;
  /* Variables to compute current speed */
  ros::Time _last_time;
  float _current_wheel_speed;
  int _last_encoder_value;
  /* Motor parameters */
  MotorSimulator* _pMotor;
  int _motor_id;
  float _max_angular_speed;
  /* Geometric parameters */
  float _L, _R;
  /* PID variables */
  ros::Time _PID_last_time;
  double _errSum, _lastErr;
  double _Kp, _Ki, _Kd;
  double _windup;

#ifdef PID_MAN_TUNE
  ros::Subscriber _debug_Kp_sub;
  ros::Subscriber _debug_Ki_sub;
  ros::Subscriber _debug_Kd_sub;
  ros::Subscriber _debug_windup_sub;
#endif

};//End of class MotorController

/* Interrupt (like CTRL+C) callback to set every PWM at 0 before ending */
void motor_ctrl_c_handler(int s)
{
  std::cout << "Caught signal " << s << std::endl;
  exit(1);
}
