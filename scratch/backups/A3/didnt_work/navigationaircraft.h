#ifndef NAVIGATION_AIRCRAFT_H
#define NAVIGATION_AIRCRAFT_H

#include <cmath>
#include <deque>
#include "simulator.h"

class NavigationAircraft
{
private:
    double linear_velocity_;
    double angular_velocity_;
    double angle_of_orientation_;
    double offset_angle_;
    double x_position_;
    double y_position_;

public:
    NavigationAircraft();
    ~NavigationAircraft();
    void getOrientation(std::vector<Pose>, Pose);
    void aircraftNavigate();
    void checkAirSpace(const std::shared_ptr<Simulator> &, Pose);
    double getLinearVelocity();
    double getAngularVelocity();
};

#endif //NAVIGATION_AIRCRAFT_H