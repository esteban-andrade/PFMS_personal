#ifndef NAVIGATIONINTERFACE_H
#define NAVIGATIONINTERFACE_H
/**
 * @file NavigationInterface.h
 * @author Esteban Andrade
 * @brief The NavigationInterface will aid to build the navigation class with standard navigation functions
 * @version 0.1
 * @date 2020-05-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "simulator.h"
#include "types.h"
/**
 * @brief The NavigationInterface class will aid to create subsequent navigation child classes
 * 
 */
class NavigationInterface
{
    /**
     * @publicsection
     * 
     */
public:
    /**
 * @brief Construct a new Navigation Interface object of abstract class
 * 
 */
    NavigationInterface(){};
    /**
     * @brief Destroy the Navigation Interface object of abstract class
     * 
     */
    ~NavigationInterface(){};
    /**
     * @brief Get the Angular Speed object in rad/s
     * 
     * @return double of Angular Speed
     */
    virtual double getAngularSpeed() = 0;
    /**
     * @brief Get the Linear Velocity object in m/s
     * 
     * @return double of Linear Velocity
     */
    virtual double getLinearVelocity() = 0;
    /**
     * @brief Get the Current Pose object the pose object in space
     * 
     * @return Pose of position and orientation
     */
    virtual Pose getCurrentPose() = 0;
};

#endif //NAVIGATIONINTERFACE_H