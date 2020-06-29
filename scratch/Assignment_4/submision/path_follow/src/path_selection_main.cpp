/**
 * @file path_selection_main.cpp
 * @author Esteban Andrade
 * @brief Main function used to start the node path_follow_select_path. This will allow to select the points in the OG MAP
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "ros/ros.h"
#include "path_selection_handler.h"
#include "path_generation.h"

int main(int argc, char **argv)
{

  /**
   * @brief init ROS function and name node to path_follow_select_path
   * 
   */
  ros::init(argc, argv, "path_follow_select_path");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on teh function desired
   */
  std::shared_ptr<PathSelection> ps(new PathSelection(nh)); //!< Create shared pointer
  std::thread t1(&PathSelection::separateThread, ps);       //!< Start thread

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();
  t1.join();

  return 0;
}
