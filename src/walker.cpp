/* Copyright (c) 2019 Ethan Quist
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file walker.cpp
 *
 * @brief This is a ros code to let the turtlebot roam around the world
 *
 * @author Ethan Quist
 *
 * @copyright Ethan Quist
 *
 *
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <gazebo_msgs/DeleteModel.h>
#include <stdlib.h>

/**
 * @brief This class handles publishing velocity and subscribing to Bumper Events
 *
 */
class myTurtleClass {
 public:
  myTurtleClass() {
    // Publishing velocity commands
    pub_ = n_.advertise < geometry_msgs::Twist
        > ("/mobile_base/commands/velocity", 1);

    // Subscribing to Bump Events
    sub_ = n_.subscribe("mobile_base/events/bumper", 1,
                        &myTurtleClass::callback, this);

    //service client
    client_ = n_.serviceClient < gazebo_msgs::DeleteModel
        > ("/gazebo/delete_model");

  }

  /**
   * @brief this is the callback function for the subscriber node
   *
   * @param a pointer to the subscribed message
   *
   * @return none
   */
  void callback(const kobuki_msgs::BumperEvent::ConstPtr& input) {
    bool throwaway;
    std::string modelName("cube_20k_1");
    ROS_INFO_STREAM("Uh oh!!");

    //delete a box
    //RemoveModel(const std::string &cube_20k_1);
    gazebo_msgs::DeleteModel dmsrv;
    dmsrv.request.model_name = "cube_20k_1";
    if (client_.call(dmsrv)) {
      //something
    }

    // When bumpevent happens turn the robot instead of going straight
    geometry_msgs::Twist output;
    // make turtle turn
    // don't move straight
    output.linear.x = 0;
    // do turn
    output.angular.z = 3;
    pub_.publish(output);

  }

 private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceClient client_;
};
// End of class myTurleClass

int main(int argc, char **argv) {
  // Initiate ROS
  ros::init(argc, argv, "walker");

  // Create an object
  myTurtleClass myTurtleObject;

  // Run when not bumping
  ros::NodeHandle nh;  // creating the node handler

  // publishing the velocity to the turtlebot to move straight
  ros::Publisher pub = nh.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 10);

  ros::Rate rate(2);
  while (ros::ok()) {
    geometry_msgs::Twist msg;
    // move straight
    msg.linear.x = 0.1;
    // don't turn
    msg.angular.z = 0;

    // publish the velocity to the turtlebot
    pub.publish(msg);

    ROS_INFO_STREAM(
        "Onward!");

    ros::spinOnce();
    rate.sleep();
  }

  ros::spin();

  return 0;
}
