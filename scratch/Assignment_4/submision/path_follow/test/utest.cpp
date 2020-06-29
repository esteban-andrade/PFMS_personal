/**
 * @file utest.cpp
 * @author Esteban Andrade
 * @brief Unit test 
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <gtest/gtest.h>
#include <climits>
#include <vector>

#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/LaserScan.h>
#include "../src/image_processing.h"
#include "../src/obstacle_detection.h"

TEST(GeneratedImage, ConnectionFree)
{

  //! The below code tests line connectivity
  //! On a occupied, free, and partial free OgMap

  double mapSize = 20.0;
  double resolution = 0.1;

  int pixels = (int)mapSize / resolution;

  // Create an OgMap which is occupied (all zeros)
  cv::Mat image = cv::Mat::zeros(pixels, pixels, CV_8UC1);

  // Let's check map size compared to allocation, just in case
  ASSERT_EQ(pixels, image.rows);
  ASSERT_EQ(pixels, image.cols);

  // Let's check the map is allocated all zeros
  ASSERT_EQ(0, image.at<uchar>(0, 0));

  // o------> j
  // |
  // |
  // v i
  //

  //! Create an object of image processing as we will use the public function of that object
  //! to run tests against
  ImageProcessing imageProcessing(image, true);

  cv::Point orig(pixels / 4, pixels / 2);
  cv::Point dest(3 * pixels / 4, pixels / 2);

  // Should the image allow the origin and destination to be connected in a straight line?
  ASSERT_FALSE(imageProcessing.checkConnectivity(orig, dest));

  //Now make OgMap which is free space
  image.setTo(cv::Scalar(255, 255, 255));
  imageProcessing.setImage(image);

  // Does the image now allow the origin and destination to be connected with a straight line?
  ASSERT_TRUE(imageProcessing.checkConnectivity(orig, dest));

  // Draw a black line down the middle
  cv::Point pt1(pixels / 2, 0);
  cv::Point pt2(pixels / 2, pixels);
  cv::line(image, pt1, pt2, cv::Scalar(0, 0, 0), 1);

  // Does the image now allow the origin and destination to be connected with a straight line?
  ASSERT_FALSE(imageProcessing.checkConnectivity(orig, dest));
}

TEST(OGMapSample_1, ConnectionFree)
{
  std::string path = ros::package::getPath("path_follow");
  // Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/samples/";
  // The file is called smiley_og.png
  std::string file = path + "path_test_1.png";

  cv::Mat image = cv::imread(file, 0); //The zero forces it to a grayscale image (single channel, such as OgMap)

  ASSERT_EQ(image.rows, image.cols); // Test if aquare image
  ASSERT_EQ(image.channels(), 1);    // Test if aingle channel

  //! Create an object of image processing as we will use the public function of that object to run tests against
  ImageProcessing imageProcessing(image, true);

  //Questions? Why do we have the {} around the below statements?
  {
    cv::Point org(79, 100);
    cv::Point dest(144, 102);

    ASSERT_TRUE(imageProcessing.checkConnectivity(org, dest));
  }

  {
    cv::Point org(79, 100);
    cv::Point dest(50, 102);

    ASSERT_FALSE(imageProcessing.checkConnectivity(org, dest));
  }

  {
    cv::Point org(79, 100);
    cv::Point dest(80, 131);

    ASSERT_FALSE(imageProcessing.checkConnectivity(org, dest));
  }
  {
    cv::Point org(100, 100);
    cv::Point dest(103, 66.5);

    ASSERT_TRUE(imageProcessing.checkConnectivity(org, dest));
  }
}
TEST(OGMapSample_2, ConnectionFree)
{
  std::string path = ros::package::getPath("path_follow");
  // Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/samples/";
  // The file is called smiley_og.png
  std::string file = path + "path_test_2.png";

  cv::Mat image = cv::imread(file, 0); //The zero forces it to a grayscale image (single channel, such as OgMap)

  ASSERT_EQ(image.rows, image.cols); // Test if aquare image
  ASSERT_EQ(image.channels(), 1);    // Test if aingle channel

  //! Create an object of image processing as we will use the public function of that object to run tests against
  ImageProcessing imageProcessing(image, true);

  {
    cv::Point org(71.7, 110.2);
    cv::Point dest(22.2, 106.9);

    ASSERT_TRUE(imageProcessing.checkConnectivity(org, dest));
  }

  {
    cv::Point org(71.7, 110.2);
    cv::Point dest(98.9, 109.5);

    ASSERT_FALSE(imageProcessing.checkConnectivity(org, dest));
  }
  {
    cv::Point org(81.5, 36);
    cv::Point dest(71.7, 110.2);

    ASSERT_TRUE(imageProcessing.checkConnectivity(org, dest));
  }
  {
    cv::Point org(81.5, 36);
    cv::Point dest(158, 135);

    ASSERT_FALSE(imageProcessing.checkConnectivity(org, dest));
  }
}

TEST(No_Close_Obstacle, Obstacle_Detection)
{
  ObstacleDetection obtacleDetection;
  std::string path = ros::package::getPath("path_follow");
  // Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/bag/";
  // The file is called smiley_og.png
  std::string file = path + "test_laser_reading_1.bag";

  //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
  rosbag::Bag bag;
  bag.open(file); // BagMode is Read by default
  sensor_msgs::LaserScan::ConstPtr laserScan;

  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    laserScan = m.instantiate<sensor_msgs::LaserScan>();
    if (laserScan != nullptr)
    {
      // Now we have a laserScan so we can proceed
      break;
    }
  }

  bag.close();
  ASSERT_NE(laserScan, nullptr);
  {
    bool free = obtacleDetection.detectObtacle(laserScan);
    ASSERT_EQ(true, free);
  }
}

TEST(Close_Obstacle, Obstacle_Detection)
{
  ObstacleDetection obtacleDetection;
  std::string path = ros::package::getPath("path_follow");
  // Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/bag/";
  // The file is called smiley_og.png
  std::string file = path + "test_laser_reading_2.bag";

  //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
  rosbag::Bag bag;
  bag.open(file); // BagMode is Read by default
  sensor_msgs::LaserScan::ConstPtr laserScan;

  for (rosbag::MessageInstance const m : rosbag::View(bag))
  {
    laserScan = m.instantiate<sensor_msgs::LaserScan>();
    if (laserScan != nullptr)
    {
      // Now we have a laserScan so we can proceed
      break;
    }
  }

  bag.close();
  ASSERT_NE(laserScan, nullptr);
  {
    bool detected = obtacleDetection.detectObtacle(laserScan);
    ASSERT_EQ(false, detected);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
