#include <vector>
#include <cmath>
#include "simulator.h"
#include "Navigation.h"

static const double MAX_LINEAR_VELOCITY = 900;
static const double TERMINAL_LINEAR_VELOCITY = 50;
static const double GRAVITY_FORCE = 9.81;
static double SIDE_LENGTH_SPACE = 8000 / 2;
static const double MAX_G = 6 * GRAVITY_FORCE;
static const double MAX_ANGULAR_SPEED = MAX_G / TERMINAL_LINEAR_VELOCITY;
static const double MIN_ANGULAR_SPEED = MAX_G / MAX_LINEAR_VELOCITY;
static const double LOCATION_BUFFER = 100;

Navigation::Navigation()
{
}
Navigation::~Navigation()
{
}

double Navigation::getDistanceFromBase(Pose Pose)
{
    double distance = sqrt(pow(Pose.position.x, 2) + pow(Pose.position.y, 2));
    return distance;
}
void Navigation::navigate(const std::shared_ptr<Simulator> &sim, Pose friendly)
{

    friendly = sim->getFriendlyPose();
    bool inside_airspace = false;
    angular_speed_ = 0;
    linear_velocity_ = MAX_LINEAR_VELOCITY;
    //sim->controlFriendly(linear_velocity_, angular_speed_);
    double upper_limit = 270 + friendly.orientation;
    double lower_limit = 270 - friendly.orientation;
    if (fabs(friendly.position.x) >= SIDE_LENGTH_SPACE || fabs(friendly.position.y) >= SIDE_LENGTH_SPACE)
    {
        if (!(inside_airspace))
        {
            if (sim->getFriendlyPose().orientation > upper_limit || sim->getFriendlyPose().orientation < lower_limit)
            {
                if ((friendly.position.x > 0 && friendly.position.y > 0) || (friendly.position.x < 0 && friendly.position.y < 0))
                {
                    angular_speed_ = MAX_ANGULAR_SPEED;
                    linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
                    //sim->controlFriendly(linear_velocity_, angular_speed_);
                }
                else if ((friendly.position.x < 0 && friendly.position.y > 0) || (friendly.position.x > 0 && friendly.position.y < 0))
                {
                    angular_speed_ = -MAX_ANGULAR_SPEED;
                    linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
                }
            }

            else
            {
                angular_speed_ = 0;
                linear_velocity_ = MAX_LINEAR_VELOCITY;
                inside_airspace = true;
            }
        }
    }

    sim->controlFriendly(linear_velocity_, angular_speed_);
}
double Navigation::getOrientation(const std::shared_ptr<Simulator> &sim, Pose Pose)
{
    Pose = sim->getFriendlyPose();
    oriententation_ = Pose.orientation;
    return oriententation_;
}
double Navigation::getLinearVelocity()
{
    return linear_velocity_;
}
double Navigation::getAngularSpeed()
{
    return angular_speed_;
}
