#ifndef PATH_GENERATION_H
#define PATH_GENERATION_H
/**
 * @file path_generation.h
 * @author Esteban Andrade
 * @brief This Class was used with multiple intention that include Receive the Poses. Analyse the poses and navigate the robot
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

//ROS Message Types
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"

//ROS-OpenCV Tools for Image Manipulation
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <atomic>
#include <cmath>
#include "sensor_msgs/LaserScan.h"
#include "image_handler.h"
#include "geometry_msgs/Pose2D.h"
#include "image_processing.h"
#include "obstacle_detection.h"
/**
 * @brief This Class was used with multiple intention that include Receive the Poses. Analyse the poses and navigate the robot
 * 
 */
class PathGeneration : public ImageHandler
{
    ros::NodeHandle nh_;                                     //!< ROS node handle
    image_transport::ImageTransport it_;                     //!< image transport for the received path
    ros::Subscriber sub1_;                                   //!< subscriber  for odom
    ros::Subscriber sub2_;                                   //!< subscriber for published path
    image_transport::Subscriber sub3_;                       //!< subcriber for map image full
    ros::Subscriber sub4_;                                   //!< subcriber for base_scan
    cv_bridge::CvImagePtr cvPtr_;                            //!< image bridge
    ros::ServiceClient client_;                              //!< ros client service
    image_transport::Publisher image_pub_;                   //!< image publisher for path_following
    ros::Publisher cmd_vel_pub_;                             //!< publisher for cmd_vel
    geometry_msgs::Twist control_;                           //!< message to get the twist control
    int count_;                                              //!< integer for count of poses
    double resolution_;                                      //!< image resolution
    double mapSize_;                                         //!< map size
    std::deque<std::pair<double, double>> position_target_;  //!< container for position target
    std::deque<std::pair<double, double>> position_tracker_; //!< container for position tracker for points
    std::deque<double> target_angle_;                        //!< container for target angle of point
    double robot_yaw;                                        //!< robot angle of orientation (yaw)
    bool reached_target_;                                    //!< bool to check if reached target
    int elements_;                                           //!< count bumber of received poses
    float laser_reading_;                                    //!< float for laser reading value
    unsigned int index_;                                     //!< index of target position
    bool valid_target_;                                      //!< bool to check if target is valid
    double laser_readings_;                                  //!< double to  hold the target laser reading
    bool obstacle_detected_;                                 //!< bool to detect an obstacles with laser
    bool readjust_;                                          //!< bool to check is robot needs to readjust
    bool purePursuit;                                        //!< bool to check if paramater pure pusuit was passed

    /**
 * @brief Struct used to managed the generated poses
 * 
 */
    struct PoseArrayGeneration
    {
        std::mutex mtx;                 //!< mutex to lock data
        std::atomic<bool> received;     //!< bool to indicate data to be sent
        geometry_msgs::PoseArray poses; //!< series of poses as PoseArray
        geometry_msgs::Pose robot_pose; //!< the vehicle pose
        std::atomic<double> mapSize;    //!< mapSize
        std::atomic<double> resolution; //!< resolution
    };

public:
    /**
 * @brief Object that will be used to manage Image data
 * 
 */
    ImageData imageDataRecieved_;
    /**
     * @brief Object used to handle all the Poses data
     * 
     */
    PoseArrayGeneration pathArrayGen_;
    /**
     * @brief Object used to process the Images
     * 
     */
    ImageProcessing imageProcessing_;
    /**
     * @brief Object used to detect obstacles with the laser
     * 
     */
    ObstacleDetection obstacleDetection_;
    /**
     * @brief Construct a new Path Generation object
     * 
     */
    PathGeneration(ros::NodeHandle);
    /**
     * @brief Destroy the Path Generation object
     * @param[in] ros::NodeHandle to be used to ROS connectivity, 
     */
    ~PathGeneration();
    /**
     * @brief  Call back function that will receive the selected poses. 
     * @param[in]  geometry_msgs::PoseArrayConstPtr. Pointer to the Pose Array message
     */
    void pathCallBack(const geometry_msgs::PoseArrayConstPtr &);
    /**
     * @brief The odom call back will be used to retrieve the odometry and location of the robot in space
     * @param[in] nav_msgs::OdometryConstPtr. Pointer to the odometry message
     */
    void odomCallBack(const nav_msgs::OdometryConstPtr &);
    /**
     * @brief Call back function used to retrieve the image of the laser View
     * @param[in] sensor_msgs::ImageConstPtr. Pointer to Image msg
     * 
     */
    void imageCallBack(const sensor_msgs::ImageConstPtr &);
    /**
     * @brief Call back function used to get the laser Readings
     * @param[in] sensor_msgs::LaserScanConstPtr. Pointer used to laser Scan message
     */
    void laserCallBack(const sensor_msgs::LaserScanConstPtr &);
    /**
     * @brief Fuction that will make a new thread that will get the points and and draw then in an image
     * 
     */
    void receivedPath();
    /**
     * @brief Function that will make a new thread that will print the points status and analyse if they are either free, ocupied or unkown. 
     * 
     */
    void printPath();
    /**
     * @brief Fucntion that will control the robot based on the analysed points
     * 
     */
    void navigation();
};

#endif //PATH_GENERATION_H