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
static const double LOCATION_BUFFER = 200;

Navigation::Navigation()
{
    friendly_inside_ = true;
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
    current_pose_ = friendly;
    double upper_limit;
    double lower_limit;
    if (oriententation_ > M_PI)
    {
        lower_limit = oriententation_ - M_PI - M_PI * 0.1;
        upper_limit = oriententation_ - M_PI + M_PI * 0.1;
    }
    else
    {
        lower_limit = oriententation_ + M_PI - M_PI * 0.1;
        upper_limit = oriententation_ + M_PI + M_PI * 0.1;
    }
    if ((fabs(friendly.position.x) >= SIDE_LENGTH_SPACE - LOCATION_BUFFER || fabs(friendly.position.y) >= SIDE_LENGTH_SPACE - LOCATION_BUFFER))
    {
        friendly_inside_ = false;
        if (friendly.position.x >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && oriententation_ >= 1.5 * M_PI ||
            friendly.position.x <= -SIDE_LENGTH_SPACE - LOCATION_BUFFER && oriententation_ <= M_PI ||
            friendly.position.y >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && (oriententation_ <= 0.5 * M_PI || oriententation_ >= 1.5 * M_PI) ||
            friendly.position.y <= -SIDE_LENGTH_SPACE - LOCATION_BUFFER && (oriententation_ <= 1.5 * M_PI || oriententation_ >= 1.9 * M_PI))
        {
            angular_speed_ = -MAX_ANGULAR_SPEED;
            linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
        }
        else if (friendly.position.x >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && oriententation_ < 0.5 * M_PI ||
                 friendly.position.x <= -SIDE_LENGTH_SPACE - LOCATION_BUFFER && oriententation_ > M_PI ||
                 friendly.position.y >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && (oriententation_ > 0.5 * M_PI || oriententation_ <= 1.5 * M_PI) ||
                 friendly.position.y <= -SIDE_LENGTH_SPACE - LOCATION_BUFFER && (oriententation_ > 1.5 * M_PI || oriententation_ <= 0.5 * M_PI))
        {
            angular_speed_ = MAX_ANGULAR_SPEED;
            linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
        }
        if (!(friendly_inside_))
        {
            if (friendly.orientation > upper_limit || friendly.orientation < lower_limit)
            {
                linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
            }
            else
            {
                angular_speed_ = 0;
                linear_velocity_ = MAX_LINEAR_VELOCITY;
                friendly_inside_ = true;
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
Pose Navigation::getCurrentPose()
{
    return current_pose_;
}

bool Navigation::isFriendlyInAirSpace()
{
    return friendly_inside_;
}

void Navigation::purePursuit(const std::shared_ptr<Simulator> &sim, const RangeBearingStamped &bogie_target)
{

    if (bogie_target.bearing < 2 * M_PI - 0.2 && bogie_target.bearing >= 2 * M_PI - 0.5)
    {
        linear_velocity_ = MAX_LINEAR_VELOCITY;
        angular_speed_ = -MAX_ANGULAR_SPEED;
    }
    else if (bogie_target.bearing < 0.5 && bogie_target.bearing >= 0.2)
    {
        linear_velocity_ = MAX_LINEAR_VELOCITY;
        angular_speed_ = -MAX_ANGULAR_SPEED;
    }
    else if (bogie_target.bearing < 2 * M_PI - 0.5 && bogie_target.bearing >= M_PI)
    {
        linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
        angular_speed_ = -MAX_ANGULAR_SPEED;
    }
    else if (bogie_target.bearing >= 0.5 && bogie_target.bearing < M_PI)
    {
        linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
        angular_speed_ = MAX_ANGULAR_SPEED;
    }
    else if (bogie_target.bearing < 0.2 || bogie_target.bearing > 2 * M_PI - 0.2 || bogie_target.range > 3000)
    {
        linear_velocity_ = MAX_LINEAR_VELOCITY;
        angular_speed_ = 0;
    }
    //sim->controlFriendly(linear_velocity_, angular_speed_);
}