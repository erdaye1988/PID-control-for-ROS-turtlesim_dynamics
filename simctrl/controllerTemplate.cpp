/* This a template for simple close-loop feedback controller in ROS
   with one subscriber and one publisher
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

class SubscribeAndPublish()
{
public:
  SubscribeAndPublish()
  {
    // Topic you what to publish
    pub = nh.advertise<MESSAGE_TYPE>("TOPIC_NAME", 1000);

    // Topic you what to subscribe
    sub = nh.subscribe("TOPIC_NAME", 1000,
          &SubscribeAndPublish::callback, this);

    void callback(const SUB_MESSAGE_TYPE& input)
    {
      PUB_MESSAGE_TYPE output;

      // Your Control Algorithm

      pub.publish(output);
    }
  }

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;

}; // End of class

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "Controller");

    // Create an object of SubscribAndPublish that will take care of everything
    SubscribeAndPublish controller;

    ros::spin();

    return 0;
}
