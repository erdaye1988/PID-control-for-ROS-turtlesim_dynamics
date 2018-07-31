/*
this program publish user defined tracking trajectories

*/

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include "PIDctrl.h"
#include <stdlib.h>

// mode 0: move to a randomly generated point
// mode 1: move to randomly generated points, changed each 10 secs
// mode 2: move to randomly generated point, and surveil it in a circular patern

void Mode0(ros::NodeHandle nh, ros::Publisher pub)
{
  turtlesim::Pose goal;
  double max_length = 11;

  // seed the random number generator
  srand(time(0));
  // Initialize the goal
  goal.x = double(rand()) / double(RAND_MAX) * max_length;
  goal.y = double(rand()) / double(RAND_MAX) * max_length;
  // change each 0.5 secs
  ros::Rate rate(2);
  while(ros::ok())
  {
    // Publish the message
    pub.publish(goal);
    // display ROS_INFO_STREAM
    // wait until it's time for another iteration
    rate.sleep();
  }
}

void Mode1(ros::NodeHandle nh, ros::Publisher pub)
{
  turtlesim::Pose goal;
  double max_length = 11;

  // seed the random number generator
  srand(time(0));
  // change each 10 secs
  ros::Rate rate(0.1);
  while(ros::ok())
  {
    // create motion output
    goal.x = double(rand()) / double(RAND_MAX) * max_length;
    goal.y = double(rand()) / double(RAND_MAX) * max_length;

    // Publish the message
    pub.publish(goal);

    // display ROS_INFO_STREAM
    // wait until it's time for another iteration
    rate.sleep();
  }
}

void Mode2(ros::NodeHandle nh, ros::Publisher pub)
{
  turtlesim::Pose goal;
  double center_x, center_y;
  double max_length = 11;
  double initial_time = ros::Time::now().toSec();
  double current_time;
  double angle = 0;

  // seed the random number generator
  srand(time(0));
  // Initialize the goal and center
  center_x = double(rand()) / double(RAND_MAX) * max_length;
  center_y = double(rand()) / double(RAND_MAX) * max_length;
  goal.x = center_x + 1*cos(angle);
  goal.y = center_y + 1*sin(angle);

  // move to assigned location
  ros::Rate rate(10);
  while(ros::ok())
  {
    current_time = ros::Time::now().toSec();
    if ((current_time - initial_time) > 15) {
      break;
    }
    // Publish the message
    pub.publish(goal);
    // wait until it's time for another iteration
    rate.sleep();
  }

  // surveil around the center
  // one step: 0.1s, 3.6 degrees
  while(ros::ok())
  {
    // compute new goal location
    angle += (3.6/180) * M_PI;
    goal.x = center_x + 1*cos(angle);
    goal.y = center_y + 1*sin(angle);
    // Publish the messages
    pub.publish(goal);
    // wait
    rate.sleep();
  }
}

int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node
  ros::init(argc, argv, "motion_plan");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_("~");
  int mode_name = 0;

  // Create a publisher object
  ros::Publisher pub = nh.advertise<turtlesim::Pose>(
          "goal", 1000);

  // Set the mode parameter to default value
  nh_private_.param<int>("mode", mode_name, 0);

  if (mode_name == 1) {
    ROS_INFO("Running Mode 1: multiple locations...");
    Mode1(nh, pub);
  }
  else if (mode_name == 2) {
    ROS_INFO("Running Mode 2: circular surveilliance...");
    Mode2(nh, pub);
  }
  else {
    ROS_INFO("Running Mode 0: single location...");
    Mode0(nh, pub);
  }


  return 0;
}
