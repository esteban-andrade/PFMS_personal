/**
 * @file obstacle_detection.cpp
 * @author Esteban Andrade
 * @brief Class that will analyse if there are obstacles with in the given conditions
 * @version 0.1
 * @date 2020-06-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "obstacle_detection.h"

static const int LASER_FIELD_OF_VIEW = 140; //!< Given FOV
static const double LASER_LIMIT = 0.2;      //!< Laser stop limit
static const double RAD_ROBOT = 0.2;        //!< Radius of the robot

/**
 * @brief Construct a new Obstacle Detection:: Obstacle Detection object
 * 
 */
ObstacleDetection::ObstacleDetection()
{
}
/**
 * @brief bolean used to detect whether an obtacle is with a specific distance and in the field of view
 * 
 * @param[in] laserScan. sensor_msgs::LaserScan::ConstPtr
 * @return true if obtacle not detected with given conditions
 * @return false if obstacle detected with the given conditions
 */
bool ObstacleDetection::detectObtacle(sensor_msgs::LaserScan::ConstPtr laserScan)
{

  double laser_reading = laserScan->range_max;                                  // get max reading
  int range_start = (laserScan->ranges.size() / 2) - (LASER_FIELD_OF_VIEW / 2); // consider star of FOV
  int range_end = (laserScan->ranges.size() / 2) + (LASER_FIELD_OF_VIEW / 2);   //consider end of FOV
  for (unsigned int i = range_start; i <= range_end; i++)
  {
    if (laserScan->ranges.at(i) < laser_reading)
    {
      laser_reading = laserScan->ranges.at(i); // Store laser reading
    }
  }
  if (laser_reading <= LASER_LIMIT + RAD_ROBOT) // check if condition is valid
  {
    return false;
  }
  else
  {
    return true;
  }
}
/**
 * @brief Function that will return the closes laser reading within that field of view
 * 
 * @param[in] laserScan. sensor_msgs::LaserScan::ConstPtr 
 * @return double laser reading 
 */
double ObstacleDetection::getLaserReading(sensor_msgs::LaserScan::ConstPtr laserScan)
{

  int range_start = (laserScan->ranges.size() / 2) - (LASER_FIELD_OF_VIEW / 2); // consider star of FOV
  int range_end = (laserScan->ranges.size() / 2) + (LASER_FIELD_OF_VIEW / 2);   //consider end of FOV
  for (unsigned int i = range_start; i <= range_end; i++)
  {
    if (laserScan->ranges.at(i) < laser_reading_)
    {
      laser_reading_ = laserScan->ranges.at(i); // store closest reading
    }
  }
  return laser_reading_; // return closes reading
}
