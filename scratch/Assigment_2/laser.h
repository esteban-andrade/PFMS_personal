#ifndef LASER_H
#define LASER_H

#include "ranger.h"

/**
 * \class Laser 
 * \brief  Laser returns NL number of measurements (distances) which are related to the specified angular resolution. Each measurement is a single point in space
 * \see ranger.h
 * \see laser.h
*/
class Laser : public Ranger
{
  /**
   * \publicsection
  */
public:
  /**
   * Default Constructor will set the laser parameters to default
   * \note Default configuration will be 1 (angular resolution = 10)
  */
  Laser();

  /**
   * \brief constructor that will set a specific configuration
   * \note there are two valid configurations: \n 1) Angular resolution = 10 \n 2) Angular resolution = 30
   * \param[in] config will be used to implement the laser configurationo  which is related to the angular resolution
  */
  Laser(int);

  /**
   * \brief will set the laser angular resolutoltion and will check it based on laser specification
   * \param[in] angular_resolution will be used to change the laser angular resolution
  */
  bool setAngularResolution(double);

  /**
   * \brief will set the laser configuration
   * \note Valid Angular Resolutions: \n 1) 10 degrees \n 2) 30 degrees
   * \param[in] config will be used to change the laser configuration
  */
  bool setLaserConfiguration(int);

  /**
   * \note there are two valid configurations: \n 1) Angular resolution = 10 \n 2) Angular resolution = 30
   * \return laser configuration
  */
  unsigned int getLaserConfiguration(void);

  /**
   * \return number of valid laser configurations
  */
  unsigned int getNumberOfLaserConfigurations(void);

  /**
   * \privatesection
  */
private:
  unsigned int laser_configuration; //! < laser current configuration
};

#endif // LASER_H
