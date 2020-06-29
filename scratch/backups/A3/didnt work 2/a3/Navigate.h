#ifndef NAVIGATE_H
#define NAVIGATE_H

#include <cmath>
#include <vector>
#include "simulator.h"

class Navigate
{
private:
    double vector_of_orientation_;
    double offset_;
    double linear_velocity_;
    double angular_velocity_;

public:
    Navigate();
    ~Navigate();
    void analyseOrientation(std::vector<Pose>, Pose);
    void navigateAircraft();
    void checkAirspace(const std::shared_ptr<Simulator> &, Pose);
    double getLinearVelocity();
    double getAngularSpeed();
};

#endif //NAVIGATE_H