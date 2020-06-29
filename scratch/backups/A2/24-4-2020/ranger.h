#ifndef RANGER_H
#define RANGER_H

#include "rangerinterface.h"
#include <string>

class Ranger : public RangerInterface
{
  public:
  //Default constructors should set all sensor attributes to a default value
  Ranger();

  //See rangerinterface.h for more information

  //Generates raw data for sensor
  std::vector<double> generateData();
  std::vector<double> getData();

  //Essential getters for obtaining internal private variables
  unsigned int getAngularResolution(void);
  int getOffset(void);
  unsigned int getFieldOfView(void);
  double getMaxRange(void);
  double getMinRange(void);
  virtual SensingMethod getSensingMethod(void); // based on the enum the ranger cant
  virtual std::string getModel();
  unsigned int getNumberSamples(void);

  //Essential setters for setting internal private variables
  bool setAngularResolution(unsigned int);
  bool setOffset(int);
  bool setFieldOfView(unsigned int);
  bool setNumberOfSamples(unsigned int);
  virtual bool setModel(std::string);
  bool setMinDistance (double);
  bool setMaxDistance(double);
  virtual SensingMethod setSensingMethod(SensingMethod);

protected:
  double offset_;
  int field_of_view_;
  int angular_resolution_;
  int number_samples_;
  std::vector<double> data_;
  SensingMethod sensing_type_;
  double min_range_;
  double max_range_;
  std::string model_;


};

#endif // RANGER_H
