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
    void navigate(const std::shared_ptr<Simulator> &, Pose);
    double getOrientation(const std::shared_ptr<Simulator> &, Pose);
    double getLinearVelocity();
    double getAngularSpeed();
    Pose getCurrentPose();
    bool isFriendlyInAirSpace();

    void purePursuit(const std::shared_ptr<Simulator> &, const RangeBearingStamped &);
};

#endif //NAVIGATION