#ifndef RANGER_H
#define RANGER_H

#include "rangerinterface.h"

class Ranger : public RangerInterface
{
  public:
  //Default constructors should set all sensor attributes to a default value
  Ranger();
  
  //Generates raw data for sensor
  std::vector<double> generateData();

  //Essential getters for obtaining internal private variables
  unsigned int getAngularResolution(void);
  unsigned int getAngularResolution(void);
  double getMaxRange(void);
  double getMinRange(void);
  SensingMethod getSensingMethod(void);

  //Essential setters for setting internal private variables
  bool setAngularResolution(unsigned int);
  bool setOffset(int);
  bool fieldOfView(int);

  protected:
  double orientation_offset_;
  int field_of_view_;
  int angular_resolution_;
  int number_samples_;
  std::vector <double> data_;
  SensingMethod sensing_type_;


};

#endif // RANGER_H
