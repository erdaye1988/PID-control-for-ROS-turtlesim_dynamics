/* This a PID  controller in ROS for turtle robot
   with one subscriber and one publisher
*/

#include "PIDctrl.h"

// default loc: [5.54 5.54]
  SubscribeAndPublish::SubscribeAndPublish()
  : nh_private_("~")
  {
    // Publisher := turtle1/cmd_wrench
    output_pub = nh.advertise<geometry_msgs::Wrench>(
          "turtle1/cmd_wrench", 1000);

    // Set a timer for the Output Publisher callback function
    // frequency: 1000 Hz
    pub_timer = nh.createTimer(ros::Duration(0.02),
                &SubscribeAndPublish::pubCallback, this);

    // Subscriber := turtle1/pose
    input_sub = nh.subscribe("turtle1/pose", 1000,
          &SubscribeAndPublish::subCallback, this);

    // Subscriber := goal
    goal_sub = nh.subscribe("goal", 1000,
               &SubscribeAndPublish::goalCallback, this);

    // Initialize the variables used for computing derivative term
    last_sub_time = ros::Time::now().toSec();
    initial_time = ros::Time::now().toSec();
    last_err.dis = 0; last_err.psi = 0;
    err_integral.dis = 0; err_integral.psi = 0;

    // Initialize the goal location
    goal.x = 5.54;
    goal.y = 5.54;

    // Initialize the error threshold
    nh_private_.param<double>("threshold", err_threshold, 0.05);

    // Set control Constant
    nh_private_.param<double>("Kp1", Kp1, 0); nh_private_.param<double>("Kp2", Kp2, 0);
    nh_private_.param<double>("Kd1", Kd1, 0); nh_private_.param<double>("Kd2", Kd2, 0);
    nh_private_.param<double>("Ki1", Ki1, 0); nh_private_.param<double>("Ki2", Ki2, 0);
  }

  SubscribeAndPublish::~SubscribeAndPublish()
  {
    output.force.x = 0;
    output.torque.z = 0;
    output_pub.publish(output);
    ros::shutdown();
  }

  void SubscribeAndPublish::subCallback(const turtlesim::Pose::ConstPtr& msgs)
  {
      // assign the callback input to private class variable "input"
      input.x = msgs->x;
      input.y = msgs->y;
      input.theta = msgs->theta;
      input.linear_velocity = msgs->linear_velocity;
      input.angular_velocity = msgs->angular_velocity;
  }

  void SubscribeAndPublish::pubCallback(const ros::TimerEvent& event)
  {
    // Your Control Algorithm
    // ------------------------ P term ---------------------------
    error err;
    double psi_g;
    //int sign;
    // compute the direction toward the goal position
    psi_g   = atan2(goal.y - input.y, goal.x - input.x);
    psi_g   = fmod(psi_g + 2*M_PI, 2*M_PI); // transform from [-pi,pi] to [0,2pi]
    //err.psi = psi_g - input.theta;
    //sign = (std::signbit(err.psi) == 0) ? 1 : -1;
    // compute the error
    err.dis = sqrt(pow(goal.x - input.x, 2) +
                   pow(goal.y - input.y, 2));
    //err.psi = (fabs(err.psi) <= M_PI) ? err.psi :
    //                sign * fmod(fabs(err.psi) - 2*M_PI, M_PI); // convert to acute angles
    err.psi = findDifference(input.theta, psi_g);

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    err.dis = err.dis * cos(err.psi);

    // ------------------------ D term --------------------------
    double current_sub_time = ros::Time::now().toSec();
    double dt = current_sub_time - last_sub_time;
    error err_derivative;
    // compute the derivative of the error
    err_derivative.dis = (err.dis - last_err.dis) / dt;
    err_derivative.psi = (err.psi - last_err.psi) / dt;
    // update
    last_sub_time = current_sub_time;
    last_err = err;

    // ---------------------- I term ----------------------
    // compute the integral of the error
    err_integral.dis += err.dis * dt;
    err_integral.psi += err.psi * dt;

    // compute output
    output.force.x  = Kp1 * err.dis + Ki1 * err_integral.dis +
                      Kd1 * err_derivative.dis;
    output.torque.z = Kp2 * err.psi + Ki2 * err_integral.psi +
                      Kd2 * err_derivative.psi;

    // publish the output
    output_pub.publish(output);

    // display turtle info
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"Time: "
      <<current_sub_time-initial_time<<"s");
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed
      << "GOAL: x: " << goal.x<< " | y: " << goal.y);
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed
      << "Pose: x: " << input.x<< " | y: " << input.y<< " | dir: "<<input.theta);
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed
      << "lin_vel: " << input.linear_velocity<< " | ang_vel: " << input.angular_velocity);
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed
      << "force_x: " << output.force.x<< " | torque_z: " << output.torque.z);
    //ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<< " P term: " << err.dis
    //  << " | D term: " << err_derivative.dis<< " | I term: " << err_integral.dis);
    ROS_INFO_STREAM("-------------------------------------------------");
  }

  void SubscribeAndPublish::goalCallback(const turtlesim::Pose::ConstPtr& msgs)
  {
    goal.x = msgs->x;
    goal.y = msgs->y;
    goal.theta = msgs->theta;
  }

// helper function
// Given two heading angles, compute the smaller angle
// between those two. Note: two angles must have the
// range : [0, 2*Pi].
double findDifference(double init_psi, double goal_psi)
{
  double err  = goal_psi - init_psi;
  int sign = (std::signbit(err) == 0) ? 1 : -1;
  err  = (fabs(err) <= M_PI) ? err :
                  sign * fmod(fabs(err) - 2*M_PI, M_PI);
  return err;
}
