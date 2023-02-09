#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>
#include <list>

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_node");

  RosbotClass rosbot;
  rosbot.move();

  list<float> list_coordinates;
  list_coordinates = rosbot.get_position_full();
  for (float coordinate : list_coordinates) {
    ROS_INFO_STREAM(coordinate << ", ");
  }

  return 0;
}

//  list<float> list_coordinates;
//   list_coordinates = rosbot.get_position_full();
//   for (float coordinate : list_coordinates) {
//     ROS_INFO_STREAM(coordinate << ", ");
//   }

// if (condition_1) {
//     statement_block_1;
// } else if (condition_2) {
//     statement_block_2;
// } else {
//     statement_block_else;
// }


