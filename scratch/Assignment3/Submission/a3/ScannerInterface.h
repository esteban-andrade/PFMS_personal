#ifndef SCANNERINTERFACE_H
#define SCANNERINTERFACE_H
/**
 * @file ScannerInterface.h
 * @author Esteban Andrade
 * @brief  ScannerInterface will be the base class for all the scanner class and can be used to make other classes
 * @version 0.1
 * @date 2020-05-26
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "simulator.h"
#include "types.h"

/**
 * @brief This ScannerInterface is an  abstract class that will be the base for multiple scanning classes 
 * @see ScannerInterface.h
 */
class ScannerInterface
{
    /**
 * @publicsection
 * 
 */
public:
    /**
 * @brief Construct a new Scanner Interface object of abstract class
 * 
 */
    ScannerInterface(){};
    /**
     * @brief Destroy the Scanner Interface object of the abstract class
     * 
     */
    ~ScannerInterface(){};
    /**
     * @brief Virtual function to reset all the acquired data
     * @note Will need to be used reset all the STL containers of data
     * 
     */
    virtual void resetData() = 0;
    /**
     * @brief Get the Object Velocity of the target as Scalar
     * @note Will get the converted Scalar velocity of the vector
     * @return std::vector<double> of velocity
     */
    virtual std::vector<double> getObjectVelocity() = 0;
    /**
     * @brief Get the Target object in x and y coordinates based on a given plane
     * @note Will  be analysed in reference to a plane of reference
     * @return std::vector<std::pair<double, double>> position 
     */
    virtual std::vector<std::pair<double, double>> getTarget() = 0;
    /**
     * @brief Get the Velocity Vector object
     * @note The velocity will be obtain in rectangular coordiantes with Vx and Vy
     * @return std::vector<std::pair<double, double>> velocityVector
     */
    virtual std::vector<std::pair<double, double>> getVelocityVector() = 0;
    /**
     * @brief Get the Predicted Target object 
     * @note Will get the predicted position in x and y 
     * @return std::vector<std::pair<double, double>> predicted_position
     */
    virtual std::vector<std::pair<double, double>> getPredictedTarget() = 0;
};

#endif //SCANNERINTERFACE_H
