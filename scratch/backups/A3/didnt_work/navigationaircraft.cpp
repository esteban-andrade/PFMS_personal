#include <cmath>
#include <vector>
#include "simulator.h"
#include "navigationaircraft.h"

NavigationAircraft::NavigationAircraft()
{
}

NavigationAircraft::~NavigationAircraft()
{
}

void NavigationAircraft::getOrientation(std::vector<Pose> boggie, Pose aircraft)
{
    double orientation_y = boggie.front().position.y - aircraft.position.y;
    double orientation_x = boggie.front().position.x - aircraft.position.x;
    angle_of_orientation_ = atan2(orientation_y, orientation_x);
    if (angle_of_orientation_ <= 0)
    {
        angle_of_orientation_ = 2 * M_PI * fabs(angle_of_orientation_);
    }
    offset_angle_ = angle_of_orientation_ - aircraft.orientation;

    y_position_ = fabs(offset_angle_);
    x_position_ = 2 * M_PI - y_position_;
}

void NavigationAircraft::aircraftNavigate()
{
    if (offset_angle_ < -0.087 && offset_angle_ > 0.087)
    {
        linear_velocity_ = 900;
        angular_velocity_ = 0.0;
    }
    else if (offset_angle_ < 0)
    {
        linear_velocity_ = 50;
        angular_velocity_ = -1.1772;
    }
    else if (offset_angle_ > 0)
    {
        linear_velocity_ = 50;
        angular_velocity_ = 1.1772;
    }
}

void NavigationAircraft::checkAirSpace(const std::shared_ptr<Simulator> &sim, Pose aircraft)
{
    if (aircraft.position.x > sim->AIRSPACE_SIZE || aircraft.position.y > sim->AIRSPACE_SIZE)
    {
        while (aircraft.orientation == -aircraft.orientation)
        {
            angular_velocity_ = 1.1772;
            linear_velocity_ = 50;
        }
        linear_velocity_ = 900;
        angular_velocity_ = 0.0;
    }
}

double NavigationAircraft::getLinearVelocity()
{
    return linear_velocity_;
}

double NavigationAircraft::getAngularVelocity()
{
    return angular_velocity_;
}