#include "path_selection_handler.h"
//#include "image_handler.h"
//ROS-OpenCV Tools for Image Manipulation
/**
 * @file path_selection_handler.cpp
 * @author Esteban Andrade
 * @brief Class that will manage and send the selected poses
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

/**
 * @brief Struct used to determine and manage poses 
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
PoseArrayData poseArrayBuffer_; //!< Object of struct PoseArrayData

/**
 * @brief Construct a new Path Selection:: Path Selection object
 * 
 * @param nh 
 */
PathSelection::PathSelection(ros::NodeHandle nh) : nh_(nh), it_(nh)
{
    sub1_ = nh.subscribe("robot_0/odom", 1000, &PathSelection::odomCallBack, this); // get robot odom
    image_transport::ImageTransport it(nh);                                         // transport image
    sub2_ = it.subscribe("map_image/full", 1, &PathSelection::imageCallBack, this); // get map image
    posePublisher_ = nh_.advertise<geometry_msgs::PoseArray>("path", 100);          // publishes created path
    nh_.getParam("/local_map/map_resolution", resolution_);                         // get image resolution
    nh_.getParam("/local_map/map_width", mapSize_);                                 // get maps size
    poseArrayBuffer_.send_ = false;                                                 // sets status to not send until poses are selected
    poseArrayBuffer_.map_size_ = mapSize_;                                          // store mapsize
    poseArrayBuffer_.resolution_ = resolution_;                                     // store resolution
}

/**
 * @brief Destroy the Path Selection:: Path Selection object
 * 
 */
PathSelection::~PathSelection()
{
    cv::destroyWindow("Path Selection");
}

/**
 * @brief function that will manage the callback reaction of the mouse click and store the poses
 * 
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 * @param param 
 */
void callBackFunction(int event, int x, int y, int flags, void *param)
{
    PoseArrayData *pose_array_buffer = ((PoseArrayData *)(param));

    if (event == cv::EVENT_LBUTTONDOWN)
    {
        std::cout << "Left Button of mouse click at Position [x,y] = [ " << x << " , " << y << " ]" << std::endl;
        geometry_msgs::Pose pose;

        double centre = (pose_array_buffer->map_size_) / 2;

        pose.position.x = (double(x) - (centre)) * pose_array_buffer->resolution_;
        pose.position.y = -(double(y) - (centre)) * pose_array_buffer->resolution_;
        pose_array_buffer->mtx.lock();

        pose_array_buffer->poses_.poses.push_back(pose);
        pose_array_buffer->mtx.unlock();
        std::cout << "pixel  position (" << x << ", " << y << ")" << std::endl;
        std::cout << "Local position (" << pose.position.x << ", " << pose.position.y << ")" << std::endl;
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        pose_array_buffer->mtx.lock();
        if (pose_array_buffer->poses_.poses.size() > 0)
        {
            pose_array_buffer->send_ = true;
            std::cout << "Right button of the mouse is clicked, Poses will be sent " << std::endl;
        }
        else
        {
            std::cout << " No poses Selected , can not be sent " << std::endl;
        }
        pose_array_buffer->mtx.unlock();
    }
}

/**
 * @brief The odom call back will be used to retrieve the odometry and location of the robot in space
 * 
 * @param msg [in]nav_msgs::OdometryConstPtr. Pointer to the odometry message
 */
void PathSelection::odomCallBack(const nav_msgs::OdometryConstPtr &msg)
{
    geometry_msgs::Pose pose = msg->pose.pose; // get robot location and odometry
    poseArrayBuffer_.mtx.lock();
    poseArrayBuffer_.robot_pose_ = pose;
    poseArrayBuffer_.mtx.unlock();
}

/**
 * @brief Call back function used to retrieve the image of the laser View
 * 
 * @param msg [in] sensor_msgs::ImageConstPtr. Pointer to Image msg
 */
void PathSelection::imageCallBack(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        if (enc::isColor(msg->encoding))
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
        else
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    imageDataBuffer_.mtx.lock();
    imageDataBuffer_.imageDeque_.push_back(cvPtr_->image);

    if ((cvPtr_->image.cols != 200) || (cvPtr_->image.rows != 200))
    {
        ROS_WARN_THROTTLE(60, "The image is not 200 x 200 what has gone wrong!");
    }
    imageDataBuffer_.timeStampDeque_.push_back(msg->header.stamp);
    if (imageDataBuffer_.imageDeque_.size() > 3)
    {
        imageDataBuffer_.imageDeque_.pop_front(); // eliminate one element from the stack
        imageDataBuffer_.timeStampDeque_.pop_front();
    }

    imageDataBuffer_.mtx.unlock();
}

/**
 * @brief function Used to generate a new thread that will execute the program and select the points. 
 * 
 */
void PathSelection::separateThread()
{
    ros::Time timeImage = ros::Time::now();
    cv::Mat image;
    count_ = 0;
    cv::namedWindow("Path Selection", CV_WINDOW_NORMAL);
    cv::waitKey(50);
    //cv::setMouseCallback("Path Selection", &callBackFunction, &poseArrayBuffer_);
    cv::setMouseCallback("Path Selection", callBackFunction, &poseArrayBuffer_);
    while (ros::ok())
    {
        // get map image
        bool image_ok = false;
        imageDataBuffer_.mtx.lock();
        if (imageDataBuffer_.imageDeque_.size() > 0)
        {
            image = imageDataBuffer_.imageDeque_.front();
            timeImage = imageDataBuffer_.timeStampDeque_.front();
            imageDataBuffer_.imageDeque_.pop_front();
            imageDataBuffer_.timeStampDeque_.pop_front();
            image_ok = true;
        }
        imageDataBuffer_.mtx.unlock();
        //check image and send poses
        if (image_ok)
        {
            cv::Mat rgbImage;
            cv::cvtColor(image, rgbImage, CV_GRAY2RGB);
            cv::circle(rgbImage, cv::Point((image.rows / 2), (image.cols / 2)), 3, CV_RGB(255, 0, 255), 2);
            if (poseArrayBuffer_.send_)
            {
                poseArrayBuffer_.mtx.lock();
                poseArrayBuffer_.poses_.header.stamp = ros::Time::now();
                posePublisher_.publish(poseArrayBuffer_.poses_);
                poseArrayBuffer_.poses_.poses.clear();
                poseArrayBuffer_.send_ = false;
                poseArrayBuffer_.mtx.unlock();
                cv::putText(rgbImage, "Sent Path.", cv::Point(30, 30),
                            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(200, 200, 250), 1, CV_AA);
                cv::imshow("Path Selection", image);
                cv::waitKey(3000); // waits for 3s so we can visualise this
            }
            else
            {
                cv::imshow("Path Selection", image);
                cv::waitKey(5);
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    std::cout << __func__ << " thread terminated" << std::endl;
}
