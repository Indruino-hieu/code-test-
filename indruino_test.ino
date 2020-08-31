#define USE_USBCON
#include "config.h"
//#include "control_motor.h"

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  //tTime[6] = millis();
}

ros::NodeHandle  nh;

std_msgs::Float32 str_msg;
ros::Publisher Value("value", &str_msg);

void setup()
{
  nh.initNode();
  nh.advertise(Value);
  nh.subscribe(cmd_vel_sub);
}

void loop()
{
  str_msg.data = goal_velocity_from_cmd[LINEAR];
  Value.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}