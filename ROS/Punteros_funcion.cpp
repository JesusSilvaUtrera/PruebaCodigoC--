#include "rosbot_control/rosbot_class.h"
#include <iostream>
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_node");

  RosbotClass rosbot;
  rosbot.move_forward(4);
//   float array_distances[2]; 
//   array_distances[0] = rosbot.get_laser(659);
//   array_distances[1] = rosbot.get_laser(70);
    
//   cout << "The wall is at " << array_distances[0] << " m on the left" << endl;
//   cout << "The wall is at " << array_distances[1] << " m on the right" << endl;
  float *laser_value = rosbot.get_laser_full();
  ROS_INFO_STREAM("Laser values: ");
  for (int i=0;i<=719;i++) {
    ROS_INFO_STREAM(*laser_value);
    laser_value++;
  }
    
  return 0;