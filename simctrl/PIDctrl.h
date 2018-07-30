/* This a PID  controller in ROS for turtle robot
   with one subscriber and one publisher
*/

#ifndef PIDCTRL_H
#define PIDCTRL_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <math.h>
#include <iomanip>

// struct of goal location
struct goalstr
{
    double x;
    double y;
    double psi;
};
// struct of error
struct error
{
   double dis;
   double psi;
};

class SubscribeAndPublish
{
public:
  SubscribeAndPublish();
  ~SubscribeAndPublish();
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_;
  // ROS Topic Publishers and Timers
  ros::Publisher output_pub;
  ros::Timer pub_timer;
  // ROS Topic Subscribers
  ros::Subscriber input_sub;
  ros::Subscriber goal_sub;

  // messages to publish or subscribe on topics
  turtlesim::Pose input;
  geometry_msgs::Wrench output;
  turtlesim::Pose goal;

  // Callback Functions
  void subCallback(const turtlesim::Pose::ConstPtr& msgs);
  void pubCallback(const ros::TimerEvent& event);
  void goalCallback(const turtlesim::Pose::ConstPtr& msgs);

  // private variables for the PID algorithm
  error last_err;
  error err_integral;
  double initial_time;
  double last_sub_time;
  // PID constants
  double Kp1, Kp2;
  double Ki1, Ki2;
  double Kd1, Kd2;
  // I term tolerance threshold
  double err_threshold;
}; // end of class

double findDifference(double init_psi, double goal_psi);


#endif
