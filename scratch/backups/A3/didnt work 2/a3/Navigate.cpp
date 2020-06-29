#include <cmath>
#include <vector>
#include "simulator.h"
#include "types.h"
#include "Navigate.h"

static const double MAX_LINEAR_VELOCITY = 900;
static const double TERMINAL_LINEAR_VELOCITY = 50;
static const double GRAVITY_FORCE = 9.81;
static double SIDE_LENGTH_SPACE = 8000 / 2;
static const double MAX_G = 6 * GRAVITY_FORCE;
static const double MAX_ANGULAR_SPEED = MAX_G / TERMINAL_LINEAR_VELOCITY;
static const double MIN_ANGULAR_SPEED = MAX_G / MAX_LINEAR_VELOCITY;
static const double LOCATION_BUFFER = 100;

Navigate::Navigate()
{
}
Navigate::~Navigate()
{
}
void Navigate::analyseOrientation(std::vector<Pose> bogie_predicted, Pose friendly)
{
    double y_orientation = bogie_predicted.back().position.y - friendly.position.y;
    double x_orientation = bogie_predicted.back().position.x - friendly.position.x;
    vector_of_orientation_ = atan2(y_orientation, x_orientation);
    if (vector_of_orientation_ <= 0)
    {
        vector_of_orientation_ = 2 * M_PI - fabs(vector_of_orientation_);
    }
    offset_ = vector_of_orientation_ - friendly.orientation;
}
void Navigate::navigateAircraft()
{
    if (offset_ < 0.087 && offset_ > -0.087)
    {
        linear_velocity_ = MAX_LINEAR_VELOCITY;
        angular_velocity_ = 0;
    }
    else if (offset_ < 0)
    {
        linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
        angular_velocity_ = -MAX_ANGULAR_SPEED;
    }
    else if (offset_ > 0)
    {
        linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
        angular_velocity_ = MAX_ANGULAR_SPEED;
    }
}
void Navigate::checkAirspace(const std::shared_ptr<Simulator> &, Pose friendly)
{
    if (fabs(friendly.position.x) > SIDE_LENGTH_SPACE || fabs(friendly.position.y) > SIDE_LENGTH_SPACE)
    {
        double orientation = friendly.orientation;
        while (orientation == -orientation)
        {
            angular_velocity_ = MAX_ANGULAR_SPEED;
            linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
        }
        angular_velocity_ = 0;
        linear_velocity_ = MAX_LINEAR_VELOCITY;
    }
}
double Navigate::getLinearVelocity()
{
    return linear_velocity_;
}
double Navigate::getAngularSpeed()
{
    return angular_velocity_;
}
