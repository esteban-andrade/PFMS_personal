#ifndef PATH_SELECTION_H
#define PATH_SELECTION_H
/**
 * @file image_handler.h
 * @author Esteban Andrade
 * @brief These class will allow to inherit different parameters that will be used for image and poses handling.
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <sstream>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <atomic>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
//ROS Message Types
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"
#include "sensor_msgs/image_encodings.h"
//ROS-OpenCV Tools for Image Manipulation
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//ROS Service
#include "a4_setup/RequestGoal.h"

namespace enc = sensor_msgs::image_encodings;
/**
 *  @brief These class will allow to inherit different parameters that will be used for image and poses handling.
 * 
 */
class ImageHandler
{
public:
    /**
 * @brief Construct a new Image Handler object
 * 
 */
    ImageHandler(){};
    /**
     * @brief Destroy the Image Handler object
     * 
     */
    ~ImageHandler(){};

protected:
    /**
 * @brief Struct use for image handling 
 * 
 */
    struct ImageData
    {
        std::deque<cv::Mat> imageDeque_;       //!< container that will hold images
        std::deque<ros::Time> timeStampDeque_; //!< container that will hold the time
        std::mutex mtx;                        //!< mutex for data protection and synchronization
    };
    /**
     * @brief struct use for Poses handling
     * 
     */
    struct PoseArrayData
    {
        std::mutex mtx;                  //!< mutex for data protection and synchronization
        std::atomic<bool> send_;         //!< atamic bool used for node connectivity.
        geometry_msgs::PoseArray poses_; //!< message use for poses target
        geometry_msgs::Pose robot_pose_; //!< message to handle robot pose
        std::atomic<double> map_size_;   //!< size of the map
        std::atomic<double> resolution_; //!< total image resolution
    };
};
#endif // PATH_SELECTION_H