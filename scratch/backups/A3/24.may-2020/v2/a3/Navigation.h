#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <vector>
#include "simulator.h"
#include "NavigationInterface.h"

class Navigation : public NavigationInterface
{
private:
    double oriententation_;
    double linear_velocity_;
    double angular_speed_;
    double tangent_distance_;
    Pose current_pose_;
    bool friendly_inside_;

public:
    Navigation();
    ~Navigation();
    double getDistanceFromBase(Pose);
    void navigate(Pose &, Pose);
    double getOrientation(Pose, Pose);
    double getLinearVelocity();
    double getAngularSpeed();
    Pose getCurrentPose();
    bool isFriendlyInAirSpace();
    void purePursuit(const RangeBearingStamped &, Pose);
};

#endif //NAVIGATION