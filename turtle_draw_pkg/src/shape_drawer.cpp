#include "shape_drawer.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

ShapeDrawer::ShapeDrawer() {  
  //Publisher for the movement data, size of publishing queue 
  turtlesim_pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
}

void CircleDrawer::draw() {
//Loop publishing messages 10 times a second
  ros::Rate loop_rate(10);

  geometry_msgs::Twist moveForwardMsg; //Provides messages for common movements
  moveForwardMsg.linear.x = 3.0;       //Linear speed
  moveForwardMsg.angular.z = 0;        //Angular speed
  geometry_msgs::Twist rotateMsg;
  rotateMsg.angular.z = (2*M_PI) / 5;  //Rotates turtle a certain angle
//Keeps repeating movements until ros stops
  while (ros::ok()) {
    turtlesim_pub.publish(moveForwardMsg);
    loop_rate.sleep();
    turtlesim_pub.publish(rotateMsg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
//Square function
void SquareDrawer::draw() {
  ros::Rate loop_rate(1);

  geometry_msgs::Twist moveForwardMsg;
  moveForwardMsg.linear.x = 2.0;
  moveForwardMsg.angular.z = 0;
  geometry_msgs::Twist rotateMsg;
  rotateMsg.angular.z = (2*M_PI) / 4;

  for (int iters=0; iters<5;iters++) {
    turtlesim_pub.publish(moveForwardMsg);
    loop_rate.sleep();
    turtlesim_pub.publish(rotateMsg);
    loop_rate.sleep();
  }
}
//Triangle function
void TriangleDrawer::draw() {
  ros::Rate loop_rate(1);

  geometry_msgs::Twist moveForwardMsg;
  moveForwardMsg.linear.x = 2.0;
  moveForwardMsg.angular.z = 0;
  geometry_msgs::Twist rotateMsg;
  rotateMsg.angular.z = (2*M_PI) / 3;

  for (int itert=0; itert<4;itert++) {
    turtlesim_pub.publish(moveForwardMsg);
    loop_rate.sleep();
    turtlesim_pub.publish(rotateMsg);
    loop_rate.sleep();
  }
}
//Pentagon function
void PentagonDrawer::draw() {
  ros::Rate loop_rate(1);

  geometry_msgs::Twist moveForwardMsg;
  moveForwardMsg.linear.x = 2.0;
  moveForwardMsg.angular.z = 0;
  geometry_msgs::Twist rotateMsg;
  rotateMsg.angular.z = (2*M_PI) / 5;

  for (int iterp=0; iterp<6;iterp++) {
    turtlesim_pub.publish(moveForwardMsg);
    loop_rate.sleep();
    turtlesim_pub.publish(rotateMsg);
    loop_rate.sleep();
  }
}