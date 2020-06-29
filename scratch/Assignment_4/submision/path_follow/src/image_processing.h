#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H
/**
 * @file image_processing.h
 * @author Esteban Andrade
 * @brief Class that will allow to process and analyse images
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <opencv2/opencv.hpp>
/**
 * @brief Struct for pixel handling
 * 
 */
struct pixel
{
  int x; //!< x location
  int y; //!< y location
};
/**
 * @brief  Class that will allow to process and analyse images
 * 
 */
class ImageProcessing
{

public:
  /*! @brief Default ImageProcessing constructor.
   *
   */
  ImageProcessing();

  /*! @brief ImageProcessing constructor.
   *
   *  @param image OpenCV image to be analysed
   */
  ImageProcessing(cv::Mat image, bool debug);

  /*! @brief Checks wether the origin and destination can be connected with a line, such that line only goes over free space
   *
   *  @param[in]    cv::Point oirgin - point of origin [i,j] in image coordinates
   *  @param[in]    cv::Point destination - point of destination in image coordinates
   *  @return bool  The points can be connected with a line which only goes over free space
   * @note The value for grey pixels was also considered as valid as it can change status as the robot gets closer to the target point
   */
  bool checkConnectivity(cv::Point origin, cv::Point destination);

  /*! @brief Sets the debug and visualisation information on(true)/off(false)
     *
     *  @param[in]    bool debug - enable/disable debug
     */
  void setDebug(bool debug);

  /*! @brief Sets the image
      *
      *  @param[in]    cv::Mat image - new image
      */
  void setImage(cv::Mat image);
  /**
   * @brief will allow to pass an image and return processes image
   * 
   * @param   image 
   * @return cv::Mat 
   */
  cv::Mat processImage(cv::Mat image);
  /**
   * @brief struct to handle pixel location 
   * 
   */
  pixel pix;

private:
  cv::Mat image_; /*!< OpenCV image structure */
  const cv::Scalar CLR_GOAL = cv::Scalar(0, 0, 255);
  const cv::Scalar CLR_ORIGIN = cv::Scalar(0, 255, 0);
  bool debug_; /*!< Debug switch */
};

#endif // IMAGE_PROCESSING_H
