/**
 * @file plot_path_main.cpp
 * @author Esteban Andrade
 * @brief Main function that will start the node path_follow_draw_path. This node will have 3 threads for multiple processes.
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "ros/ros.h"
#include "path_generation.h"
int main(int argc, char **argv)
{

    /**
     * @brief ROS init that will create node path_follow_draw_path
     * 
     */
    ros::init(argc, argv, "path_follow_draw_path");

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

    std::shared_ptr<PathGeneration> pg(new PathGeneration(nh)); //!< Create Shared Pointer
    std::thread t2(&PathGeneration::receivedPath, pg);          //!< Create Thread one that will receive the send poses
    std::thread t3(&PathGeneration::printPath, pg);             //!< Create Thread two that will print the path ana analyse the point status
    std::thread t4(&PathGeneration::navigation, pg);            //!< Create Thread three that will allow the robot to  navigate to multiple locations

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

    t2.join();
    t3.join();
    t4.join();

    return 0;
}
