#include <vector>
#include <cmath>
#include "simulator.h"
#include "Navigation.h"

static const double MAX_LINEAR_VELOCITY = 900;                            // Max Linear Velocity Constant m/s
static const double TERMINAL_LINEAR_VELOCITY = 50;                        // Terminal Linear Velocity m/s
static const double GRAVITY_FORCE = 9.81;                                 // Gravitational Force m/s^2
static double SIDE_LENGTH_SPACE = 8000 / 2;                               // Map Size Side constant
static const double MAX_G = 6 * GRAVITY_FORCE;                            // Constrain to MAX G
static const double MAX_ANGULAR_SPEED = MAX_G / TERMINAL_LINEAR_VELOCITY; // Calculate Maximun angular speed rad/s
static const double MIN_ANGULAR_SPEED = MAX_G / MAX_LINEAR_VELOCITY;      // Calculate Minumum angular speed rad/s
static const double LOCATION_BUFFER = 200;                                // Location Buffer for Navigation
static const int LOOK_A_HEAD_DISTANCE_THRESHOLD = 500;                    // Lookahead Distance threshold for purePursuit
static const int LOOK_A_HEAD_CHASE = 2000;                                // Lookahead Distance for purePursuit
static const double K = 0.8;                                              // Gain for controller for navigation

Navigation::Navigation()
{
    friendly_inside_ = true;                //Assume friendly is inside the airspace
    angular_speed_ = 0;                     //Set friendly Angular speed to zero
    linear_velocity_ = MAX_LINEAR_VELOCITY; //Set linear Speed to max
}
Navigation::~Navigation()
{
}

double Navigation::getDistanceFromBase(Pose Pose)
{
    double distance = sqrt(pow(Pose.position.x, 2) + pow(Pose.position.y, 2)); //Calculate the tangent distance of the friendly relative to the base
    tangent_distance_ = distance;                                              // assigned calcuted distance
    return distance;
}
void Navigation::navigate(Pose &friendly_simulator, Pose friendly)
{
    friendly = friendly_simulator;          //get friendly orientation in real time
    current_pose_ = friendly;               //get current friendly pose
    oriententation_ = friendly.orientation; //get friendly orientation
    double upper_limit;                     // get limits for steering parameters
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
    angular_speed_ = 0;
    linear_velocity_ = MAX_LINEAR_VELOCITY;
    if ((fabs(friendly.position.x) >= SIDE_LENGTH_SPACE - LOCATION_BUFFER || fabs(friendly.position.y) >= SIDE_LENGTH_SPACE - LOCATION_BUFFER)) // control that the friendly does not exceed the limits
    {
        friendly_inside_ = false; // if limit exceeded switch boolean to false
        if (!(friendly_inside_))
        {
            if (oriententation_ > upper_limit || oriententation_ < lower_limit) //control limits to allow correct steering based on  position in plane
            {
                if ((friendly.position.x > 0 && friendly.position.y > 0) || (friendly.position.x < 0 && friendly.position.y < 0)) // steer counterclockwise
                {
                    angular_speed_ = MAX_ANGULAR_SPEED;
                    linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
                }
                else if ((friendly.position.x < 0 && friendly.position.y > 0) || (friendly.position.x > 0 && friendly.position.y < 0)) //steer clockwise
                {
                    angular_speed_ = -MAX_ANGULAR_SPEED;
                    linear_velocity_ = TERMINAL_LINEAR_VELOCITY;
                }
            }
        }
    }
    else
    {
        friendly_inside_ = true; // if condition is not met aircraft is still in the airspace
    }
}
double Navigation::getOrientation(Pose Pose_reference, Pose Pose) //calculate the current pose based on another reference position
{
    Pose = Pose_reference;              // update the pose based on the pose reference
    oriententation_ = Pose.orientation; // update orientation based on new pose
    return oriententation_;
}
double Navigation::getLinearVelocity() // get linear velocity
{
    return linear_velocity_;
}
double Navigation::getAngularSpeed() // get angular speed
{
    return angular_speed_;
}
Pose Navigation::getCurrentPose() // get current pose
{
    return current_pose_;
}

bool Navigation::isFriendlyInAirSpace() // return condition if friendly is in airspace
{
    return friendly_inside_;
}

void Navigation::purePursuit(const RangeBearingStamped &bogie_target, Pose friendly) // pure pursuit algorithm
{
    double angle, target_x, target_y, look_ahead_distance, x1, y1, a, b, c, gama, d, x, r; // variables place holders
    angle = bogie_target.bearing;                                                          //get bearing angle in reference to bogie
    x1 = bogie_target.range * cos(bogie_target.bearing);                                   //get new position in the x direction in reference to the friendly
    y1 = bogie_target.range * sin(bogie_target.bearing);                                   //get new position in the y direction in reference to the friendly
    look_ahead_distance = bogie_target.range + LOOK_A_HEAD_DISTANCE_THRESHOLD;             // set a look a head distance based on the range
    target_x = look_ahead_distance * cos(bogie_target.bearing);                            // get new target position in x
    target_y = look_ahead_distance * sin(bogie_target.bearing);                            // get new target position in x
    a = -tan(angle);                                                                       //equation ax+by+c
    b = 1;
    c = tan(angle) * friendly.position.x - friendly.position.y;
    d = fabs(a * x1 + b * y1 + c) / sqrt(pow(a, 2) + pow(b, 2));                             // get the point line distace
    x = fabs(a * target_x + b * target_y + c) / sqrt(pow(a, 2) + pow(b, 2));                 // calculate look ahead point
    r = pow(look_ahead_distance, 2) / (2 * x);                                               // get the wo sidesof pie shape with arc
    gama = (2 * x) / pow(look_ahead_distance, 2);                                            // get gamma angle
                                                                                             // angular speed = gama * linearspeed
    if (friendly_inside_ == true)                                                            // check that the friendly is in the airspace
    {                                                                                        // will check the angles in order to steer the friendly with a PI controller
        if (bogie_target.bearing < 2 * M_PI - 0.2 && bogie_target.bearing >= 2 * M_PI - 0.5) // check if values is with in thershold 1 positive wise
        {                                                                                    // Turn fast toward look-ahead point direction
            linear_velocity_ = MAX_LINEAR_VELOCITY * K;                                      // velocity will control it  with a gain
            angular_speed_ = -MIN_ANGULAR_SPEED / K;                                         // angular speed will have a controller and gain
        }
        else if (bogie_target.bearing < 0.5 && bogie_target.bearing >= 0.2)
        {                                               // Turn fast toward look-ahead point direction
            linear_velocity_ = MAX_LINEAR_VELOCITY * K; // velocity will control it  with a gain
            angular_speed_ = MIN_ANGULAR_SPEED / K;     // angular speed will have a controller and gain
        }
        else if (bogie_target.bearing < 2 * M_PI - 0.5 && bogie_target.bearing >= M_PI)
        {                                                    //Turn slowly toward look-ahead point direction
            linear_velocity_ = TERMINAL_LINEAR_VELOCITY / K; // velocity will control it  with a gain
            angular_speed_ = -MAX_ANGULAR_SPEED * K;         // angular speed will have a controller and gain
        }
        else if (bogie_target.bearing >= 0.5 && bogie_target.bearing < M_PI)
        {                                                    //turn slowly toward look-ahead point direction
            linear_velocity_ = TERMINAL_LINEAR_VELOCITY / K; // velocity will control it  with a gain
            angular_speed_ = MAX_ANGULAR_SPEED * K;          // angular speed will have a controller and gain
        }
        else if (bogie_target.bearing < 0.2 || bogie_target.bearing > 2 * M_PI - 0.2 || bogie_target.range > LOOK_A_HEAD_CHASE + LOOK_A_HEAD_DISTANCE_THRESHOLD) //do PurePursuit
        {
            linear_velocity_ = MAX_LINEAR_VELOCITY;   // set to max speed
            angular_speed_ = gama * linear_velocity_; //control angular speed with gamma
            if (angular_speed_ > MAX_ANGULAR_SPEED)   //check that if does not exceed the max angular speed
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
        friendly_inside_ = false; // if not positive set friendly inside to false
    }
}