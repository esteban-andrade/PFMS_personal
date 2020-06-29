#ifndef PATH_SELECTION_HANDLER_H
#define PATH_SELECTION_HANDLER_H
#include "image_handler.h"
/**
 * @file path_selection_handler.h
 * @author Esteban Andrade
 * @brief This class is the one used for selecting the points in an OG map to be sent to be analysed.
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */
//ROS-OpenCV Tools for Image Manipulation
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

/**
 * @brief This class is the one used for selecting the points in an OG map to be sent to be analysed.
 * 
 */
class PathSelection : public ImageHandler
{
    ros::NodeHandle nh_;                 //!< ros Node Handle
    image_transport::ImageTransport it_; //!< image transport for node handle
    ros::Subscriber sub1_;               //!< subcriber for odom
    image_transport::Subscriber sub2_;   //!< subscriber map_image
    ros::Publisher posePublisher_;       //!< publisher of geometry_msgs::PoseArray in path
    cv_bridge::CvImagePtr cvPtr_;        //!< image pointer bridge

    int count_;         //!<
    double resolution_; //!<
    double mapSize_;    //!<

public:
    /**
 * @brief Construct a new Path Selection object
 * @param[in] ros::NodeHandle to be used to ROS connectivity, 
 */
    PathSelection(ros::NodeHandle);
    /**
     * @brief Destroy the Path Selection object
     * 
     */
    ~PathSelection();
    /**
     * @brief The odom call back will be used to retrieve the odometry and location of the robot in space
     * @param[in] nav_msgs::OdometryConstPtr. Pointer to the odometry message
     */
    void odomCallBack(const nav_msgs::OdometryConstPtr &);
    /**
     * @brief Call back function used to retrieve the image of the laser View
     * @param[in] sensor_msgs::ImageConstPtr. Pointer to Image msg
     */
    void imageCallBack(const sensor_msgs::ImageConstPtr &);
    /**
     * @brief function Used to generate a new thread that will execute the program and select the points. 
     * @note Use Left click on the image to select the points. Use Right click to send the poses. 
     */
    void separateThread();
    /**
     * @brief Object used to handle all image data
     * 
     */
    ImageData imageDataBuffer_;
};

#endif //PATH_SELECTION_HANDLER_H