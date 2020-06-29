//ROS-OpenCV Tools for Image Manipulation
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
/**
 * @file path_generation.cpp
 * @author Esteban Andrade
 * @brief Class that will analyse the given poses and navigate the robot
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "path_generation.h"
namespace enc = sensor_msgs::image_encodings;

static const double SCALE = 3.0;             //!< Scale given for the gain
static const double K = 2.0;                 //!< controller K
static const double KI = pow(K, -1);         //!< Controller KI
static const double MAX_SPEED = 0.5;         //!< Set MAXIMUM SPEED
static const double STEERING_SPEED = 0.2;    //!< Set Acceptable steering speed
static const double ANGLE_TOLERANCE = 0.087; //!< Set angle tolarance
static const double DIST_TOLERANCE = 0.1;    //!< Set distance tolarance
static const double LASER_LIMIT = 0.2;       //!< set Laser Limit
static const double RAD_ROBOT = 0.2;         //!< set Robot Radius
static const double LASER_UPPER_LIMIT = 1.5; //!< set Laser Upper limit for steering

/**
 * @brief Construct a new Path Generation:: Path Generation object
 * @note The node is subscribing to robot_0. This could be remmaped while executing rosrun
 * 
 * @param nh 
 */
PathGeneration::PathGeneration(ros::NodeHandle nh) : nh_(nh), it_(nh)
{
    sub1_ = nh_.subscribe("robot_0/odom", 1000, &PathGeneration::odomCallBack, this);     // get robot odom
    image_transport::ImageTransport it(nh);                                               // transport imahe
    sub3_ = it.subscribe("map_image/full", 1, &PathGeneration::imageCallBack, this);      // get map image
    sub4_ = nh_.subscribe("robot_0/base_scan", 10, &PathGeneration::laserCallBack, this); // receive laser scan
    sub2_ = nh_.subscribe("path", 10, &PathGeneration::pathCallBack, this);               // receive path
    image_pub_ = it_.advertise("map_image/path_following", 1);                            //publish on RQT
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 1);             //publish control of velocities
    //Below get's parameters from parameter server
    nh_.getParam("/local_map/map_resolution", resolution_);
    nh_.getParam("/local_map/map_width", mapSize_);

    // We store information allowing to convert from global to local
    pathArrayGen_.received = false;         // get path status
    pathArrayGen_.mapSize = mapSize_;       // get map size
    pathArrayGen_.resolution = resolution_; // get image resolution

    ros::NodeHandle pn("~");
    pn.param<bool>("purePursuit", purePursuit, false); // get node parameter to pure pursuit
}

/**
 * @brief Destroy the Path Generation:: Path Generation object
 * 
 */
PathGeneration::~PathGeneration()
{
    cv::destroyWindow("Recieved Path");
}

/**
 * @brief Path call back function that will receive the sent poses
 * 
 * @param msg 
 */
void PathGeneration::pathCallBack(const geometry_msgs::PoseArrayConstPtr &msg)
{
    ROS_INFO("NEW PATH RECIEVED");
    ROS_INFO_STREAM("Recieved Path has " << msg->poses.size() << " points");
    // reset parameters
    pathArrayGen_.mtx.lock();
    pathArrayGen_.poses.poses.clear();
    position_target_.clear();
    target_angle_.clear();
    position_tracker_.clear();
    reached_target_ = false;
    index_ = 0;
    readjust_ = false;

    for (unsigned int i = 0; i < msg->poses.size(); i++)
    {
        pathArrayGen_.poses.poses.push_back(msg->poses.at(i));
    }
    pathArrayGen_.received = true;
    pathArrayGen_.mtx.unlock();
}
/**
 * @brief Call back function that will get the robot position in space
 * 
 * @param msg 
 */
void PathGeneration::odomCallBack(const nav_msgs::OdometryConstPtr &msg)
{
    geometry_msgs::Pose pose = msg->pose.pose; // get robot poses
    pathArrayGen_.mtx.lock();
    pathArrayGen_.robot_pose = pose;
    pathArrayGen_.mtx.unlock();
}
/**
 * @brief Call back function that will get image of the OG MAP
 * 
 * @param msg 
 */
void PathGeneration::imageCallBack(const sensor_msgs::ImageConstPtr &msg)
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

    imageDataRecieved_.mtx.lock();
    imageDataRecieved_.imageDeque_.push_back(cvPtr_->image);

    if ((cvPtr_->image.cols != 200) || (cvPtr_->image.rows != 200))
    {
        ROS_WARN_THROTTLE(60, "The image is not 200 x 200 what has gone wrong!");
    }
    imageDataRecieved_.timeStampDeque_.push_back(msg->header.stamp);

    if (imageDataRecieved_.imageDeque_.size() > 2)
    {
        imageDataRecieved_.imageDeque_.pop_front();     // avoid overstack of images
        imageDataRecieved_.timeStampDeque_.pop_front(); // avoid overstack of timers
    }
    imageDataRecieved_.mtx.unlock();
}

/**
 * @brief Function that will get the laser reading 
 * 
 * @param msg 
 */
void PathGeneration::laserCallBack(const sensor_msgs::LaserScanConstPtr &msg)
{
    obstacle_detected_ = obstacleDetection_.detectObtacle(msg); // get parameter of obstacle detection
    laser_readings_ = obstacleDetection_.getLaserReading(msg);  // get laser reading
}

/**
 * @brief Function that will Analyse the given points and check if they can be reachable. Also it will plot the status of the points either completed, current, or remaining 
 * 
 */
void PathGeneration::printPath()
{
    ros::Time timeImage = ros::Time::now();
    cv::Mat image;

    while (ros::ok())
    {
        imageDataRecieved_.mtx.lock();
        if (imageDataRecieved_.imageDeque_.size() > 0)
        {
            image = imageDataRecieved_.imageDeque_.front();
            timeImage = imageDataRecieved_.timeStampDeque_.front();
        }
        imageDataRecieved_.mtx.unlock();

        if (!image.empty())
        {
            cv_bridge::CvImage cv_image;
            cv::cvtColor(image, cv_image.image, CV_GRAY2RGB);
            for (unsigned int i = 0; i < position_tracker_.size(); i++)
            {
                // adjust data from global to pixels
                double offset = (mapSize_) / 2;
                auto x_target = ((position_tracker_.at(i).first - pathArrayGen_.robot_pose.position.x) / resolution_) + offset;
                auto y_target = (-(position_tracker_.at(i).second - pathArrayGen_.robot_pose.position.y) / resolution_) + offset;

                if (i == index_)
                {
                    cv::Point org(100, 100);
                    cv::Point pt(x_target, y_target);
                    cv::circle(cv_image.image, pt, 3, CV_RGB(0, 255, 0), -1); // draw green point
                    imageProcessing_.setImage(image);

                    // check if it can connect the robot with the target point
                    valid_target_ = imageProcessing_.checkConnectivity(org, pt);
                    if (!valid_target_)
                    {
                        ROS_INFO_STREAM("Invalid Point " << index_ << " located in Occupied Space. Point " << index_ << " ignored.");
                        position_target_.pop_front();
                        position_tracker_.pop_front();
                    }
                }
                else if (i < index_)
                {
                    cv::Point pt(x_target, y_target);
                    cv::circle(cv_image.image, pt, 3, CV_RGB(0, 0, 255), -1); // draw blue points
                }
                else
                {
                    cv::Point pt(x_target, y_target);
                    cv::circle(cv_image.image, pt, 3, CV_RGB(255, 0, 0), -1); // draw red points
                }

                cv::Point robot((image.cols / 2), (image.rows / 2));
                cv::circle(cv_image.image, robot, 3, CV_RGB(255, 0, 255), -1); // draw robot position
            }

            cv_image.encoding = "bgr8";
            cv_image.header = std_msgs::Header();
            image_pub_.publish(cv_image.toImageMsg());
        }
    }
}

/**
 * @brief Function that will navigate and steer the robot. Based on a given parameter it will either spot turns or pure pursuit
 * 
 */
void PathGeneration::navigation()
{
    while (ros::ok())
    {
        while (position_target_.size() > 0 && valid_target_)
        {
            //get navigation parameters
            robot_yaw = tf::getYaw(pathArrayGen_.robot_pose.orientation);
            double x_dist = (position_target_.front().first - pathArrayGen_.robot_pose.position.x);
            double y_dist = (position_target_.front().second - pathArrayGen_.robot_pose.position.y);
            double angle = atan2(y_dist, x_dist);
            double linear_dist = sqrt((pow(x_dist, 2)) + (pow(y_dist, 2)));
            double gamma = 2 * x_dist / pow(linear_dist, 2);
            target_angle_.push_back((angle - robot_yaw));
            double gain = (SCALE * fabs(target_angle_.front())) / (2 * M_PI);
            //check if robot reached the target
            if (!reached_target_)
            {
                //checkif pure pursuit was passed.
                if (!purePursuit)
                {
                    if ((target_angle_.back()) > ANGLE_TOLERANCE)
                    {
                        control_.linear.x = 0;
                        control_.angular.z = STEERING_SPEED;
                        reached_target_ = false;
                    }
                    else if ((target_angle_.back()) < -ANGLE_TOLERANCE)
                    {
                        control_.linear.x = 0;
                        control_.angular.z = -STEERING_SPEED;
                        reached_target_ = false;
                    }

                    else if (((pathArrayGen_.robot_pose.position.x <= position_target_.front().first + DIST_TOLERANCE) && (pathArrayGen_.robot_pose.position.y <= position_target_.front().second + DIST_TOLERANCE)) &&
                             ((pathArrayGen_.robot_pose.position.x >= position_target_.front().first - DIST_TOLERANCE) && (pathArrayGen_.robot_pose.position.y >= position_target_.front().second - DIST_TOLERANCE)))
                    {
                        control_.linear.x = 0;
                        control_.angular.z = 0;

                        ROS_INFO_STREAM(" robot Position [x,y]= [ " << pathArrayGen_.robot_pose.position.x << " ] , [ " << pathArrayGen_.robot_pose.position.y << " ]");
                        reached_target_ = true;
                    }
                    else
                    {

                        double vel = KI * linear_dist;
                        double angular_vel = K * target_angle_.back();
                        control_.linear.x = KI * vel;
                        control_.angular.z = angular_vel;
                        if (vel > MAX_SPEED)
                        {
                            control_.linear.x = MAX_SPEED;
                        }
                        else if (angular_vel > MAX_SPEED)
                        {
                            angular_vel = MAX_SPEED;
                        }
                    }
                }
                else if (purePursuit)
                {
                    reached_target_ = false;
                    control_.angular.z = sqrt(gain * fabs(gamma));
                    if ((target_angle_.back()) < 0)
                    {
                        control_.angular.z = -control_.angular.z;
                    }

                    control_.linear.x = fabs(control_.angular.z / gamma) * (1 / gain);
                    if (((pathArrayGen_.robot_pose.position.x <= position_target_.front().first + DIST_TOLERANCE) && (pathArrayGen_.robot_pose.position.y <= position_target_.front().second + DIST_TOLERANCE)) &&
                        ((pathArrayGen_.robot_pose.position.x >= position_target_.front().first - DIST_TOLERANCE) && (pathArrayGen_.robot_pose.position.y >= position_target_.front().second - DIST_TOLERANCE)))
                    {
                        control_.linear.x = 0;
                        control_.angular.z = 0;

                        ROS_INFO_STREAM(" robot Position [x,y]= [ " << pathArrayGen_.robot_pose.position.x << " ] , [ " << pathArrayGen_.robot_pose.position.y << " ]");
                        reached_target_ = true;
                    }
                }
            }
            if (reached_target_)
            {
                position_target_.pop_front();
                target_angle_.clear();
                index_++;
                reached_target_ = false;
            }
            //check if obstacles are detected with the laser
            if (!obstacle_detected_)
            {
                control_.linear.x = 0;
                control_.angular.z = 0;
                position_target_.pop_front();
                target_angle_.clear();
                ROS_INFO_STREAM("Laser Reading " << laser_readings_);
                ROS_INFO_STREAM("Obstacle Detected ,Target Invalid, Moving to next point");
                readjust_ = true;
                if (position_target_.size() == 0)
                {
                    ROS_INFO_STREAM("Invalid Path, Enter new Parameters");
                    readjust_ = true;
                }
                //readjust robot position if obtacle is detected
                if (laser_readings_ <= LASER_LIMIT + RAD_ROBOT && laser_readings_ < LASER_UPPER_LIMIT && readjust_)
                {
                    control_.linear.x = -MAX_SPEED;
                }
                else if (laser_readings_ > LASER_LIMIT + RAD_ROBOT && laser_readings_ > LASER_UPPER_LIMIT)
                {
                    readjust_ = false;
                }
            }

            cmd_vel_pub_.publish(control_);
        }
    }
}

/**
 * @brief Function that will plot the points of the raw unprocessed points
 * 
 */
void PathGeneration::receivedPath()
{
    ros::Time timeImage = ros::Time::now();
    cv::Mat image;
    count_ = 0;
    cv::namedWindow("Received Path", CV_WINDOW_NORMAL);
    cv::waitKey(50);
    while (ros::ok())
    {
        //process received Data
        bool imageOk = false;
        imageDataRecieved_.mtx.lock();
        if (imageDataRecieved_.imageDeque_.size() > 0)
        {
            image = imageDataRecieved_.imageDeque_.front();
            timeImage = imageDataRecieved_.timeStampDeque_.front();
            imageDataRecieved_.imageDeque_.pop_front();
            imageDataRecieved_.timeStampDeque_.pop_front();
            imageOk = true;
        }
        imageDataRecieved_.mtx.unlock();

        if (imageOk)
        {

            if (pathArrayGen_.received)
            {
                ROS_INFO("New Path Recieved");
                cv::Mat RGB_IMage;
                cv::cvtColor(image, RGB_IMage, CV_GRAY2RGB);
                pathArrayGen_.mtx.lock();
                for (unsigned int i = 0; i < pathArrayGen_.poses.poses.size(); i++)
                {
                    geometry_msgs::Pose pose = pathArrayGen_.poses.poses.at(i);
                    ROS_INFO_STREAM(" Element on Path " << i);
                    ROS_INFO_STREAM(" Local Position [x,y]= [ " << pose.position.x << " ] , [ " << pose.position.y << " ]");
                    //convert locations and draw points without processing
                    double offset = (pathArrayGen_.mapSize * pathArrayGen_.resolution) / 2;
                    double x_position_image = pose.position.x;
                    double y_position_image = -pose.position.y;
                    double x_position = pose.position.x + pathArrayGen_.robot_pose.position.x;
                    double y_position = pose.position.y + pathArrayGen_.robot_pose.position.y;
                    ROS_INFO_STREAM(" Localized position [x,y]: [ " << x_position << " ],[ " << y_position << " ]");
                    double target_x = (x_position);
                    double target_y = (y_position);
                    position_target_.push_back(std::make_pair(target_x, target_y));
                    position_tracker_.push_back(std::make_pair(target_x, target_y));

                    double xD = (x_position_image + offset) / pathArrayGen_.resolution;
                    double yD = (y_position_image + offset) / pathArrayGen_.resolution;
                    ROS_INFO_STREAM(" Target  " << i << " position [" << target_x << ", " << target_y << "]");

                    cv::circle(RGB_IMage, cv::Point((xD), (yD)), 3, CV_RGB(255, 0, 0), -1);
                }
                pathArrayGen_.poses.poses.clear();
                pathArrayGen_.received = false;
                pathArrayGen_.mtx.unlock();
                cv::putText(RGB_IMage, "Received Raw Path", cv::Point(30, 30),
                            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, CV_RGB(200, 200, 250), 1, CV_AA);
                cv::imshow("Received Path", RGB_IMage);
            }
            cv::waitKey(5);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << __func__ << " thread terminated" << std::endl;
}
