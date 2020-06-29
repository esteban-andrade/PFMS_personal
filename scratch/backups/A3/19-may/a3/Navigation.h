#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <vector>
#include "simulator.h"

class Navigation
{
private:
    double oriententation_;
    double linear_velocity_;
    double angular_speed_;
    double tangent_distance_;

public:
    Navigation();
    ~Navigation();
    double getDistanceFromBase(Pose);
    void navigate(const std::shared_ptr<Simulator> &, Pose);
    double getOrientation(const std::shared_ptr<Simulator> &, Pose);
    double getLinearVelocity();
    double getAngularSpeed();
};

#endif //NAVIGATION