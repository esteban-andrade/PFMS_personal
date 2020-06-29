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
static const int LOOK_A_HEAD_DISTANCE_THRESHOLD = 500;
static const int LOOK_A_HEAD_CHASE = 2000;
static const double K = 0.8;

Navigation::Navigation()
{
    friendly_inside_ = true;
    angular_speed_ = 0;
    linear_velocity_ = MAX_LINEAR_VELOCITY;
}
Navigation::~Navigation()
{
}

double Navigation::getDistanceFromBase(Pose Pose)
{
    double distance = sqrt(pow(Pose.position.x, 2) + pow(Pose.position.y, 2));
    return distance;
}
void Navigation::navigate(Pose &friendly_simulator, Pose friendly)
{
    friendly = friendly_simulator;
    current_pose_ = friendly;
    oriententation_ = friendly.orientation;
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
    // if ((fabs(friendly.position.x) >= SIDE_LENGTH_SPACE - LOCATION_BUFFER || fabs(friendly.position.y) >= SIDE_LENGTH_SPACE - LOCATION_BUFFER))
    // {
    //     friendly_inside_ = false;
    //     if (friendly.position.x >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && oriententation_ >= 1.5 * M_PI ||
    //         friendly.position.x <= -SIDE_LENGTH_SPACE + LOCATION_BUFFER && oriententation_ <= M_PI ||
    //         friendly.position.y >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && (oriententation_ <= 0.5 * M_PI || oriententation_ >= 1.5 * M_PI) ||
    //         friendly.position.y <= -SIDE_LENGTH_SPACE + LOCATION_BUFFER && (oriententation_ <= 1.5 * M_PI || oriententation_ >= 1.9 * M_PI))
    //     {
    //         angular_speed_ = -MAX_ANGULAR_SPEED;
    //         linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
    //     }
    //     else if (friendly.position.x >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && oriententation_ < 0.5 * M_PI ||
    //              friendly.position.x <= -SIDE_LENGTH_SPACE + LOCATION_BUFFER && oriententation_ > M_PI ||
    //              friendly.position.y >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && (oriententation_ > 0.5 * M_PI || oriententation_ <= 1.5 * M_PI) ||
    //              friendly.position.y <= -SIDE_LENGTH_SPACE + LOCATION_BUFFER && (oriententation_ > 1.5 * M_PI || oriententation_ <= 0.5 * M_PI))
    //     {
    //         angular_speed_ = MAX_ANGULAR_SPEED;
    //         linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
    //     }
    //     if (!(friendly_inside_))
    //     {
    //         if (friendly.orientation > upper_limit || friendly.orientation < lower_limit)
    //         {
    //             linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
    //         }
    //         else
    //         {
    //             angular_speed_ = 0;
    //             linear_velocity_ = MAX_LINEAR_VELOCITY;
    //             friendly_inside_ = true;
    //         }
    //     }
    // }
    angular_speed_ = 0;
    linear_velocity_ = MAX_LINEAR_VELOCITY;
    if ((fabs(friendly.position.x) >= SIDE_LENGTH_SPACE - LOCATION_BUFFER || fabs(friendly.position.y) >= SIDE_LENGTH_SPACE - LOCATION_BUFFER))
    {
        friendly_inside_ = false;
        if (!(friendly_inside_))
        {
            if (oriententation_ > upper_limit || oriententation_ < lower_limit)
            {
                if ((friendly.position.x > 0 && friendly.position.y > 0) || (friendly.position.x < 0 && friendly.position.y < 0))
                {
                    angular_speed_ = MAX_ANGULAR_SPEED;
                    linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
                }
                else if ((friendly.position.x < 0 && friendly.position.y > 0) || (friendly.position.x > 0 && friendly.position.y < 0))
                {
                    angular_speed_ = -MAX_ANGULAR_SPEED;
                    linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
                }
                // if (friendly.position.x >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && oriententation_ >= 1.5 * M_PI ||
                //     friendly.position.x <= -SIDE_LENGTH_SPACE + LOCATION_BUFFER && oriententation_ <= M_PI ||
                //     friendly.position.y >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && (oriententation_ <= 0.5 * M_PI || oriententation_ >= 1.5 * M_PI) ||
                //     friendly.position.y <= -SIDE_LENGTH_SPACE + LOCATION_BUFFER && (oriententation_ <= 1.5 * M_PI || oriententation_ >= 1.9 * M_PI))
                // {
                //     angular_speed_ = -MAX_ANGULAR_SPEED;
                //     linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
                // }
                // else if (friendly.position.x >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && oriententation_ < 0.5 * M_PI ||
                //          friendly.position.x <= -SIDE_LENGTH_SPACE + LOCATION_BUFFER && oriententation_ > M_PI ||
                //          friendly.position.y >= SIDE_LENGTH_SPACE - LOCATION_BUFFER && (oriententation_ > 0.5 * M_PI || oriententation_ <= 1.5 * M_PI) ||
                //          friendly.position.y <= -SIDE_LENGTH_SPACE + LOCATION_BUFFER && (oriententation_ > 1.5 * M_PI || oriententation_ <= 0.5 * M_PI))
                // {
                //     angular_speed_ = MAX_ANGULAR_SPEED;
                //     linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
                // }
            }

            // else
            // {
            //     // angular_speed_ = 0;
            //     // linear_velocity_ = MAX_LINEAR_VELOCITY;
            //     friendly_inside_ = true;
            // }
        }
    }
    else
    {
        // angular_speed_ = 0;
        // linear_velocity_ = MAX_LINEAR_VELOCITY;
        friendly_inside_ = true;
    }
}
double Navigation::getOrientation(Pose Pose_reference, Pose Pose)
{
    Pose = Pose_reference;
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

void Navigation::purePursuit(const RangeBearingStamped &bogie_target, Pose friendly)
{
    double angle, target_x, target_y, look_ahead_distance, x1, y1, a, b, c, gama, d, x, r;
    angle = bogie_target.bearing;
    x1 = bogie_target.range * cos(bogie_target.bearing);
    y1 = bogie_target.range * sin(bogie_target.bearing);
    look_ahead_distance = bogie_target.range + LOOK_A_HEAD_DISTANCE_THRESHOLD;
    target_x = look_ahead_distance * cos(bogie_target.bearing);
    target_y = look_ahead_distance * sin(bogie_target.bearing);
    a = -tan(angle);
    b = 1;
    c = tan(angle) * friendly.position.x - friendly.position.y;
    d = fabs(a * x1 + b * y1 + c) / sqrt(pow(a, 2) + pow(b, 2));
    x = fabs(a * target_x + b * target_y + c) / sqrt(pow(a, 2) + pow(b, 2));
    //x = target_x;
    r = pow(look_ahead_distance, 2) / (2 * x);
    gama = (2 * x) / pow(look_ahead_distance, 2);
    gama = fabs(gama);

    //note angular speed = gama * linearspeed
    if (friendly_inside_ == true)
    {
        if (bogie_target.bearing < 2 * M_PI - 0.2 && bogie_target.bearing >= 2 * M_PI - 0.5)
        {
            linear_velocity_ = MAX_LINEAR_VELOCITY * K;
            angular_speed_ = -MIN_ANGULAR_SPEED / K;
        }
        else if (bogie_target.bearing < 0.5 && bogie_target.bearing >= 0.2)
        {
            linear_velocity_ = MAX_LINEAR_VELOCITY * K;
            angular_speed_ = MIN_ANGULAR_SPEED / K;
        }
        else if (bogie_target.bearing < 2 * M_PI - 0.5 && bogie_target.bearing >= M_PI)
        {
            linear_velocity_ = TERMINAL_LINEAR_VELOCITY / K;
            angular_speed_ = -MAX_ANGULAR_SPEED * K;
        }
        else if (bogie_target.bearing >= 0.5 && bogie_target.bearing < M_PI)
        {
            linear_velocity_ = TERMINAL_LINEAR_VELOCITY / K;
            angular_speed_ = MAX_ANGULAR_SPEED * K;
        }
        else if (bogie_target.bearing < 0.2 || bogie_target.bearing > 2 * M_PI - 0.2 || bogie_target.range > LOOK_A_HEAD_CHASE + LOOK_A_HEAD_DISTANCE_THRESHOLD)
        {
            linear_velocity_ = MAX_LINEAR_VELOCITY;
            angular_speed_ = gama * linear_velocity_;
            if (angular_speed_ > MAX_ANGULAR_SPEED)
            {
                angular_speed_ = MAX_ANGULAR_SPEED;
            }
            else if (angular_speed_ < -MAX_ANGULAR_SPEED)
            {
                angular_speed_ = -MAX_ANGULAR_SPEED;
            }
        }
    }
    else
    {
        friendly_inside_ = false;
    }
}