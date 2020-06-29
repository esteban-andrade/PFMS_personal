
#include "sample.h"

/**
 * This node shows some connections and publishing images
 */

PfmsSample::PfmsSample(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
{
  //Subscribing to odometry
  sub1_ = nh_.subscribe("odom", 1000, &PfmsSample::odomCallback, this);

  image_transport::ImageTransport it(nh);
  //Subscribing to image
  sub3_ = it.subscribe("map_image/full", 1, &PfmsSample::imageCallback, this);

  //Publishing an image ... just to show how
  image_pub_ = it_.advertise("test/image", 1);

  //Allowing an incoming service on /check_goal
  service_ = nh_.advertiseService("check_goal", &PfmsSample::requestGoal, this);

  //Below ge't parameters from parameter server
  nh_.getParam("/local_map/map_resolution", resolution_);

  count_ = 0;
}

PfmsSample::~PfmsSample()
{
}

bool PfmsSample::requestGoal(a4_setup::RequestGoal::Request &req,
                             a4_setup::RequestGoal::Response &res)
{
  //The incoming request is a Global coordinate.
  // To find information about the service execute "rossrv info a4_setup/RequestGoal"
  ROS_INFO_STREAM("requested goal [x,y,yaw]=[" << req.pose.x << "," << req.pose.y << "]");

  //! @todo : 4 - return true if the global coordinate supplied in the requst is within current OgMap
  //! (just needs to be within the map - either free / occupied or unknown)
  //!
  // Return in the res.ack
  // true : if the global coordinate is in the current OgMap
  // false : global coordinate outside of current OgMap

  // Informion you have now:
  // resolution_          - each pixel in OgMap corresponds to this many [m]
  // poseDataBuffer_      - position of Robot (refer code in seperateThread
  //                        on how to get x,y,yaw of robot
  // imageDataBuffer_     - ogMap, size can be accessed via image.rows and image.cols
  //                        (refer code in seperateThread on how to get ogMap

  //res.ack = is the global coordinate in the current OgMap;
  //ROS_INFO_STREAM("sending back response:" << res.ack);

  //Store goal points
  goalPoint_.x = req.pose.x;
  goalPoint_.y = req.pose.y;
  goalRequest_ = true;

  geometry_msgs::Pose robotPose;
  poseDataBuffer_.mtx.lock();
  if (poseDataBuffer_.poseDeq.size() > 0)
  {
    robotPose = poseDataBuffer_.poseDeq.back();
  }
  poseDataBuffer_.mtx.unlock();

  //get ogMap

  // double mapSize = resolution_;
  double mapSize;
  cv::Mat image;
  imageDataBuffer_.mtx.lock();
  if (imageDataBuffer_.imageDeq.size() > 0)
  {
    image = imageDataBuffer_.imageDeq.back();
    mapSize = static_cast<double>(image.cols);
  }
  imageDataBuffer_.mtx.unlock();

  res.ack = false;

  // check max and min y and x
  // double offset = (mapSize / resolution_) / 2;
  // double xmin = robotPose.position.x - offset;
  // double xmax = robotPose.position.x + offset;
  // double ymin = robotPose.position.y - offset;
  // double ymax = robotPose.position.y + offset;

  // double xmin = (robotPose.position.x - (mapSize / resolution_ / 2));
  // double xmax = (robotPose.position.x + (mapSize / resolution_ / 2));
  // double ymin = (robotPose.position.y - (mapSize / resolution_ / 2));
  // double ymax = (robotPose.position.y + (mapSize / resolution_ / 2));

  if (!image.empty())
  {
    double xmin = (robotPose.position.x - (mapSize * resolution_ / 2));
    double xmax = (robotPose.position.x + (mapSize * resolution_ / 2));
    double ymin = (robotPose.position.y - (mapSize * resolution_ / 2));
    double ymax = (robotPose.position.y + (mapSize * resolution_ / 2));

    // verify that point is in the limits
    if ((goalPoint_.x >= xmin) && (goalPoint_.x <= xmax) && (goalPoint_.y >= ymin) && (goalPoint_.y <= ymax))
    {
      res.ack = true;
    }
    std::cout << "[xmin,xmax]" << xmin << "," << xmax << std::endl;
    std::cout << "[ymin,ymax]" << ymin << "," << ymax << std::endl;
  }
  else
  {
    ROS_DEBUG_STREAM("No OG MAP received");
  }
  ROS_INFO_STREAM("sending back response:" << res.ack);
  return true;
}

void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  //! Let's get the pose out from odometry message
  //! REMEBER: on command line you can view entier msg as
  //! rosmsg show nav_msgs/Odometry
  poseDataBuffer_.mtx.lock();
  poseDataBuffer_.poseDeq.push_back(msg->pose.pose);
  poseDataBuffer_.timeStampDeq.push_back(msg->header.stamp);
  poseDataBuffer_.mtx.unlock();

  //! Additional question, what is managing this deque size? Should you?
}

void PfmsSample::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  //! Below code pushes the image and time to a deque, to share across threads
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
  imageDataBuffer_.imageDeq.push_back(cvPtr_->image);
  imageDataBuffer_.timeStampDeq.push_back(msg->header.stamp);
  //If the deque size is growing beyond 2 elements - remove the first element.
  if (imageDataBuffer_.imageDeq.size() > 2)
  {
    imageDataBuffer_.imageDeq.pop_front();
    imageDataBuffer_.timeStampDeq.pop_front();
  }
  imageDataBuffer_.mtx.unlock();
}

void PfmsSample::seperateThread()
{
  /**
    * The below loop runs until ros is shutdown, to ensure this thread does not remain
    * a zombie thread
    *
    * The loop locks the buffer, checks the size
    * And then pulls items: the pose and timer_t
    * You can contemplate weather these could have been combined into one ...
    */

  double yaw, x, y;
  /// The below gets the current Ros Time
  ros::Time timeOdom = ros::Time::now();
  ;
  ros::Time timeImage = ros::Time::now();
  ;
  cv::Mat image;

  //! @todo : 2 - Adjust rate limiter to run code every 5 seconds
  //!
  //! Additional Question
  //! What is rate_limiter?
  //!

  ros::Rate rate_limiter(0.2);
  while (ros::ok())
  {

    poseDataBuffer_.mtx.lock();
    if (poseDataBuffer_.poseDeq.size() > 0)
    {
      geometry_msgs::Pose pose = poseDataBuffer_.poseDeq.front();
      yaw = tf::getYaw(pose.orientation);
      x = pose.position.x;
      y = pose.position.y;
      timeOdom = poseDataBuffer_.timeStampDeq.front();
      poseDataBuffer_.poseDeq.pop_front();
      poseDataBuffer_.timeStampDeq.pop_front();
    }
    poseDataBuffer_.mtx.unlock();

    //! Lock image buffer, take one message, remove from deque and unlock it
    imageDataBuffer_.mtx.lock();
    if (imageDataBuffer_.imageDeq.size() > 0)
    {
      image = imageDataBuffer_.imageDeq.front();
      timeImage = imageDataBuffer_.timeStampDeq.front();
      imageDataBuffer_.imageDeq.pop_front();
      imageDataBuffer_.timeStampDeq.pop_front();
    }
    imageDataBuffer_.mtx.unlock();

    if (!image.empty())
    {

      cv_bridge::CvImage cv_image;
      //To draw a circle in colour we first convert the image (to )which in gray given it's an OgMap to colour)
      cv::cvtColor(image, cv_image.image, CV_GRAY2RGB);

      // o------> j
      // |
      // |
      // v i
      //

      //! @todo : 5 - Draw a circle at global location P(x,y)=(8.0,8.0) on the OgMap
      //!
      //! To do So:
      //! - Convert the global coordinate to pixel value in OgMap
      //! What is each pixel value in terms of [m]
      //! Where is (0,0) of the OgMap
      //! What pixel value is this?
      //! USE
      //! (image.cols, image.rows and resolution_ would be needed)

      // cv::Point pt(image.cols/2,image.rows/2);
      // cv::circle(cv_image.image, pt, 3, CV_RGB(0,255, 0) , 1);
      //std::cout << "This is where you should be drawing an dot" << std::endl;
      // if (goalRequest_)
      // {
      //   ROS_INFO_STREAM("Goal [x,y] " << goalPoint_.x << " , " << goalPoint_.y);
      //   auto offset_x = (image.cols) / 2;
      //   auto offset_y = (image.rows) / 2;

      //   auto x_target = (goalPoint_.x) + offset_x;
      //   auto y_target = (goalPoint_.y) + offset_y;
      //   //use this when wanting the point using the service
      //   //cv::Point pt(x_target, y_target);

      //   // this will be to only print on point x 8 y 8 in reference to the ogmao
      //   cv::Point pt(offset_x + 8, offset_x + 8);

      //   //use this when we want to print the circle using the service
      //   //ROS_INFO_STREAM("image position [" << x_target << ", " << y_target << "]");
      //   ROS_INFO_STREAM("image position [" << offset_x + 8 << ", " << offset_x + 8 << "]");
      //   cv::circle(cv_image.image, pt, 3, CV_RGB(0, 255, 0), 1);
      //   cv_image.encoding = "bgr8";
      //   cv_image.header = std_msgs::Header();
      //   image_pub_.publish(cv_image.toImageMsg());
      //   goalRequest_ = false;
      // }
      if (goalRequest_)
      {
        ROS_INFO_STREAM("Goal [x,y] " << goalPoint_.x << " , " << goalPoint_.y);
        double offset = (image.cols) / 2;

        goalPoint_.x -= x;
        goalPoint_.y -= y;
        ROS_INFO_STREAM("Local position [x,y] " << goalPoint_.x << " , " << goalPoint_.y);
        auto x_target = (goalPoint_.x / resolution_) + offset; //for image
        auto y_target = (-goalPoint_.y / resolution_) + offset;

        //use this when we want to print the circle using the service
        ROS_INFO_STREAM("image position [" << x_target << ", " << y_target << "]");
        //ROS_INFO_STREAM("image position [" << offset_x + 8 << ", " << offset_x + 8 << "]");
        cv::Point pt(x_target, y_target);
        cv::circle(cv_image.image, pt, 3, CV_RGB(0, 255, 0), -1);
        goalRequest_ = false;
        cv_image.encoding = "bgr8";
        cv_image.header = std_msgs::Header();
        image_pub_.publish(cv_image.toImageMsg());
      }
      // cv_image.encoding = "bgr8";
      // cv_image.header = std_msgs::Header();
      // image_pub_.publish(cv_image.toImageMsg());
    }
    rate_limiter.sleep();
  }
}
