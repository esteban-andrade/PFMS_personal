#ifndef NAVIGATION_H
#define NAVIGATION_H
/**
 * @file Navigation.h
 * @author Esteban Andrade
 * @brief Will helo to navigate in the airspace 
 * @version 0.1
 * @date 2020-05-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <vector>
#include "simulator.h"
#include "NavigationInterface.h"
/**
 * @brief The Navigation class will inherit from NavigationInterface and will allow to control the friendly and elaborate pure pursuit
 * 
 */
class Navigation : public NavigationInterface
{
    /**
     * @privatesection
     * 
     */
private:
    double oriententation_;   //!< Will hold the orientation of the friendly in space
    double linear_velocity_;  //!< Will hold the Linar velocity in m/s
    double angular_speed_;    //!< Will hold the Angular speed in rad/s
    double tangent_distance_; //!< Will hold the tangential distance
    Pose current_pose_;       //!< Will hold the current pose in space of the aircraft
    bool friendly_inside_;    //!< Bool to verify that the object is in the given airspace
    /**
 * @publicsection
 * 
 */
public:
    /**
 * @brief Construct a new Navigation object
 * @note Will set angular speed to zero and linear velocity to maximum, Will set to true friendly_inside_ boolean
 * 
 */
    Navigation();
    /**
     * @brief Destroy the Navigation object
     * 
     */
    ~Navigation();
    /**
     * @brief Get the Distance From Base object relative to the base
     * @param[in] Pose of the airscraft
     * @return double of tangetial distance
     */
    double getDistanceFromBase(Pose);
    /**
     * @brief Wll navigate in the given airspace and set values to verify that the friendly in the airspace
     * @note Will Consider a threshold angle and the position in space with a given buffer in order to steer the aircraft and avoid leaving the airspace
     * Will keep track of bolean of friendly_inside_
     * @param[in] Pose pass a reference pose for analysis
     * @param[in] Pose actual pose of the aircraft
     */
    void navigate(Pose &, Pose);
    /**
     * @brief Get the Orientation object of the aircraft in space
     * @param[in] Pose reference pose 
     * @param[in] Pose actual given pose 
     * @return double  of orientation
     */
    double getOrientation(Pose, Pose);
    /**
     * @brief Get the Linear Velocity object of aircraft
     * 
     * @return double Linear Velocity in m/s
     */
    double getLinearVelocity();
    /**
     * @brief Get the Angular Speed object of aircraft
     * 
     * @return double Angular Speed in rad/s
     */
    double getAngularSpeed();
    /**
     * @brief Get the Current Pose object in space
     * 
     * @return Pose of aircraft
     */
    Pose getCurrentPose();
    /**
     * @brief will be analysed in friendly aircraft is within the airspace
     * 
     * @return true friendly aircraft is inside the airspace
     * @return false friendly aircraft is not inside the airspace
     */
    bool isFriendlyInAirSpace();
    /**
     * @brief The pure pursuit algorithm will compute the angle gama in order to chase the bogies
     * @note This also implements a PI controller to facilitate the steering of the friendly
     * @param[in] RangeBearingStamped in order track the angle of the bogie
     * @param[in] Pose in order to track the position of the friendly as the new reference
     */
    void purePursuit(const RangeBearingStamped &, Pose);
};

#endif //NAVIGATION