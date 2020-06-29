/**
 * @file image_processing.cpp
 * @author Esteban Andrade
 * @brief This call will allow to test the connectivity of two point in an image and process the image itself
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "image_processing.h"
#include <image_transport/image_transport.h>
/**
 * @brief Construct a new Image Processing:: Image Processing object
 * 
 */
ImageProcessing::ImageProcessing() : debug_(false)
{
}
/**
 * @brief Construct a new Image Processing:: Image Processing object
 * 
 * @param[in] image to be analysed
 * @param[in] debug .either true or false to set the debugger
 */
ImageProcessing::ImageProcessing(cv::Mat image, bool debug) : image_(image), debug_(debug)
{
}
/**
 * @brief Will set the status of the debugger 
 * 
 * @param debug . set a Boolean
 */
void ImageProcessing::setDebug(bool debug)
{
  debug_ = debug;
}
/**
 * @brief Function that will set the image to be analysed
 * 
 * @param[in] image 
 */
void ImageProcessing::setImage(cv::Mat image)
{
  image.copyTo(image_); //This will copy the image into our member variable (image) courtesy of OpenCV
}
/**
 * @brief Function that was used to check of two points can be connected and if there are no black pixels in that connection
 * 
 * @param origin . Point
 * @param destination . Pint
 * @return true if there are  no pixels are in the connection line
 * @return false if  there are  pixels are in the connection line
 */
bool ImageProcessing::checkConnectivity(cv::Point origin, cv::Point destination)
{

  if (image_.empty())
  {
    std::cout << "No image supplied yet" << std::endl;
    return false;
  }

  if (debug_)
  {
    // Create an 8-bit 3-channel image from input image, so we can draw on it
    cv::Mat color_image;
    cv::cvtColor(image_, color_image, cv::COLOR_GRAY2BGR);
    // Draw the origin in Green (the colors are BGR so G is 255 and other two channels zero)
    cv::circle(color_image, origin, 1, CLR_ORIGIN, 2);
    // Draw destination in Red
    cv::circle(color_image, destination, 1, CLR_GOAL, 2);
    // Let's put some text, just for additional understanding
    cv::putText(color_image, "green - origin ,red - destination", cv::Point(0, 10), cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(255, 0, 0));
    // Make a window called debug, which can be adjusted in size
    cv::namedWindow("debug", cv::WINDOW_NORMAL);
    // Draw the color image on the windows called "debug"
    cv::imshow("debug", color_image);
    // wiat for a maximum of 5000ms for a user to click on window, then close window
    cv::waitKey(5000);
    cv::destroyWindow("debug");
  }

  bool connectivity = true; // create condition

  //! Create a Line Iterator  between origib abd destination
  cv::LineIterator lineIterator(image_, origin, destination); // create line iterator

  for (int i = 0; i < lineIterator.count; i++, lineIterator++)
  {
    uchar *pixel = (uchar *)*lineIterator;
    if (*pixel < 126) // 126 is implemented in order to accept grey as the state can change as it get closer to the target
    {
      connectivity = false; //check if it can connect
    }
  }
  return connectivity;
  //return true;
}
/**
 * @brief Will enhance the black pixels in the image to determine the configuration Space
 * 
 * @param image 
 * @return cv::Mat 
 */
cv::Mat ImageProcessing::processImage(cv::Mat image)
{
  cv::Mat processed_image = image;
  int rows = processed_image.cols;
  int cols = processed_image.rows;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      uchar pixel = processed_image.at<uchar>({i, j}); // get pixels location from image
      if (pixel == 0)                                  // check if black is detected
      {
        pix.x = i; // store position in x and y
        pix.y = j;
        for (int i = pix.x - 2; i < pix.x + 2; i++)
        {
          for (int j = pix.y - 2; j < pix.y + 2; j++)
          {
            processed_image.at<uchar>({i, j}) = 0; // adjust nearby pixels and inflate them to black
          }
        }
      }
    }
  }

  return processed_image;
}
