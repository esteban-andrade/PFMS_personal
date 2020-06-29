#ifndef DETECTCABINET_H
#define DETECTCABINET_H
/**
 * @file obstacle_detection.h
 * @author Esteban Andrade
 * @brief  Class that will allow to detect obstacles and return laser readings based on the FOV
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <sensor_msgs/LaserScan.h>
/**
 * @brief  Class that will allow to detect obstacles and return laser readings based on the FOV
 * 
 */
class ObstacleDetection
{
private:
  double laser_reading_; //!< laser closest laser reading

public:
  /**
 * @brief Construct a new Obstacle Detection object
 * 
 */
  ObstacleDetection();
  /**
   * @brief boolean used to detect if there is a close obtacle with in the field of view and Specific parameters
   * 
   * @param laserScan 
   * @return true if No obstruction is detected with in given thershold
   * @return false If target is with in the tolerance of field of view (obstacle detected)
   */
  bool detectObtacle(sensor_msgs::LaserScan::ConstPtr laserScan);
  /**
   * @brief Get the Laser Reading object
   * 
   * @param laserScan 
   * @return double laser reading
   */
  double getLaserReading(sensor_msgs::LaserScan::ConstPtr laserScan);
};

#endif // DETECTCABINET_H
