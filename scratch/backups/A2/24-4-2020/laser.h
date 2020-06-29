#ifndef LASER_H
#define LASER_H

#include "ranger.h"

class Laser : public Ranger
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Laser();
  Laser(int);
  bool setAngularResolution(double);
  bool setLaserConfiguration(int);
  unsigned int getLaserConfiguration(void);
  unsigned int getNumberOfLaserConfigurations(void);
 

private:
  unsigned int laser_configuration;
  
};

#endif // LASER_H
