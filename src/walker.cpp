#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <stdlib.h>

class myTurtleClass {
 public:
  myTurtleClass() {
    //Publishing velocity commands
    pub_ = n_.advertise < geometry_msgs::Twist
        > ("/mobile_base/commands/velocity", 1);

    //Subscribing to Bump Events
    sub_ = n_.subscribe("mobile_base/events/bumper", 1,
                        &myTurtleClass::callback, this);
  }

  void callback(const kobuki_msgs::BumperEvent::ConstPtr& input) {
    ROS_INFO_STREAM("Uh oh!!");
    geometry_msgs::Twist output;
    //make turtle turn
    //don't move straight
    output.linear.x = 0;
    //do turn
    output.angular.z = 3;
    pub_.publish(output);
  }

 private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};
//End of class myTurleClass

int main(int argc, char **argv) {
  //Initiate ROS
  ros::init(argc, argv, "walker");

  //Create an object
  myTurtleClass myTurtleObject;

  //Run when not bumping
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 10);

  ros::Rate rate(2);
  while (ros::ok()) {

    geometry_msgs::Twist msg;
    //move straight
    msg.linear.x = 0.1;
    //don't turn
    msg.angular.z = 0;

    pub.publish(msg);

    ROS_INFO_STREAM(
        "Onward!");

    ros::spinOnce();
    rate.sleep();
  }

  ros::spin();

  return 0;
}
