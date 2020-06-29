#ifndef SONAR_H
#define SONAR_H

#include "ranger.h"

/**
 * \class Sonar 
 * \brief Sonar returns 1 measurement (distance). The measurement is from an area of space (a cone in space). The distance returned is to the closest object within that area of space; therefore the return can be approximated as a triangle in space (an origin and two points at the return distance).
 * \see ranger.h
 * \see sonar.h
*/
class Sonar : public Ranger
{
  /**
   * \publicsection
  */
public:

  /**
  * \brief Default constructor should set all sensor attributes to a default value
  */
  Sonar();

private:
};

#endif // SONAR_H
