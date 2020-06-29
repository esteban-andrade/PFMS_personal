#ifndef RANGER_H
#define RANGER_H

#include "rangerinterface.h"
#include <string>

/**
 * \class Ranger will inherit from RangerInterface
 * \brief Ranger is the base class for all the sensors
 * \author Esteban Andrade
 * \see ranger.h
*/
class Ranger : public RangerInterface
{
  /**
   * \publicsection
  */
public:
  /**
   * Default constructors should set parameters to default for a non-specific sensor
  */
  Ranger();

  /**
   * \brief Generates raw data for sensor
   * \return vector with generated data
   */
  std::vector<double> generateData();
  /**
   * \brief returns vector of generated data
   * \note  This will need to be called after generate data in order to avoid retruen the results without regenerating data
  */
  std::vector<double> getData();

  /**Essential getters for obtaining internal private variables*/

  /**
  * \return angular resolution from sensors
  */
  unsigned int getAngularResolution(void);

  /**
   * \return offset from sensors
  */
  int getOffset(void);

  /**
   * \return field of view of sensors
  */
  unsigned int getFieldOfView(void);

  /**
   * \return maximum range from sensors
  */
  double getMaxRange(void);

  /**
   * \return minimum range from sensors
  */
  double getMinRange(void);

  /**
   * \return enum for sensing type
   * \see rangerinterface.h
  */
  virtual SensingMethod getSensingMethod(void);

  /**
   * \return string of sensor model
  */
  virtual std::string getModel();

  /**
   * \return number of samples based on sensors
   * \note the number of samples will be taken as a ratio between the field of view of a sensor over the angular resolution (fiels of view/angular resolution)
  */
  unsigned int getNumberSamples(void);

  /**Essential setters for setting internal private variables*/

  /** 
   * Will set the angular resolution for sensor 
   * \note virtual function that can be re-implemeted if neccesary in child classes
   * \param[in] angular_resolution will be set to sensor's angular resolution 
  */
  virtual bool setAngularResolution(unsigned int);

  /** 
   * Will set the offset for the sensors
   * \param[in] offset will be used to set the offset of the sensor in reference to the plane 
  */
  bool setOffset(int);

  /**
   * Will set the field of view of sensors
   * \param[in] field_of_view  will be set to sensor's field of view
  */
  bool setFieldOfView(unsigned int);

  /**
   * Will set the number of samples depending on sensors
   * \note This only has to be implemented if the umber of samples has to be a specific and non-dependent on other parameters
   * \param[in] samples will be set number of samples based on the ratio of field of view over angular resolution
  */
  bool setNumberOfSamples(unsigned int);

  /**
   * \note was set as virtual if there is need to reimplement this in chield classes
   * \param[in] model will be set to the sensor name
  */
  virtual bool setModel(std::string);

  /**
   * Will set the maximum range of sensor
   * \param[in] distance will be set to minimum range of sensor
  */
  bool setMinDistance(double);

  /**
   * Will set the minimum range for sensor
   * \param[in] distance will be set to maximun range of sensor
  */
  bool setMaxDistance(double);

  /**
   * Will set the sensing type of sensor based on enum
   * \see rangerinterface.h
   * \param[in] type will be set to the specific sensor sensing type
  */
  virtual SensingMethod setSensingMethod(SensingMethod);

  /**
 * \protectedsection
 * */
protected:
  double offset_;              //!< sensors offset
  int field_of_view_;          //!< field of view of sensors
  int angular_resolution_;     //!< angular resolution of sensors
  int number_samples_;         //!< number of samples of sensor
  std::vector<double> data_;   //!< vector of data results from sensors
  SensingMethod sensing_type_; //!< enum of sensing type of sensors
  double min_range_;           //!< sensors minimum range
  double max_range_;           //!<sensors maximum range
  std::string model_;          //!< model of sensors
};

#endif // RANGER_H
