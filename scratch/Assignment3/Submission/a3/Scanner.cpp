#include "simulator.h"
#include "types.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include "Scanner.h"

Scanner::Scanner()
{
}
Scanner::~Scanner()
{
}
void Scanner::baseScan(std::vector<RangeVelocityStamped> &base_scan_sim)
{
    base_data_scan_ = base_scan_sim;                                                                           // store data in real time of the base Scan sensors
    for (unsigned int i = 0; i < base_data_scan_.size(); i++)                                                  // loop for the elements
    {                                                                                                          // Store data in vector of paris
        base_scanner_.push_back(std::make_pair(base_data_scan_.at(i).range, base_data_scan_.at(i).timestamp)); // Store range and timestamp
    }
}
void Scanner::airCraftScan(std::vector<RangeBearingStamped> &raw_scan_sim)
{
    std::vector<RangeBearingStamped> raw_scan = raw_scan_sim;                                                   //get Real time data from friendly
    sort(raw_scan.begin(), raw_scan.end(), [](const RangeBearingStamped &lhs, const RangeBearingStamped &rhs) { // store and sort vector elements
        return lhs.range < rhs.range;                                                                           // return based on the condition of ranger distance prediction
    });

    friendly_data_scan_ = raw_scan;                                                                                      //store Scan data
    for (unsigned int i = 0; i < friendly_data_scan_.size(); i++)                                                        //Loop to stored elements
    {                                                                                                                    // store in a vector of pairs
        friendly_scanner_.push_back(std::make_pair(friendly_data_scan_.at(i).range, friendly_data_scan_.at(i).bearing)); // Store range and bearing
    }
}
void Scanner::friendlyRelativeBase(Pose &friendly_to_base)
{
    Pose Pose = friendly_to_base;                                              // placeholder to analyse friendly pose
    double distance = sqrt(pow(Pose.position.x, 2) + pow(Pose.position.y, 2)); // get the tangential distance
    double angle = Pose.orientation;                                           // get the orientation
    friendly_relative_to_base_.push_back(std::make_pair(distance, angle));     // store the distance and angle
}
std::vector<RangeVelocityStamped> Scanner::getBaseScanResults()
{
    return base_data_scan_; // return base scan results
}
std::vector<RangeBearingStamped> Scanner::getFriendlyScanResults()
{
    return friendly_data_scan_; // return friendly scan results
}
void Scanner::determineTarget(Pose pose_sim, std::vector<RangeBearingStamped> friendly_scan)
{
    Pose pose = pose_sim;                                                                                   //Place holder for real time data from target pose of friendly
    double friendly_orientation = pose.orientation;                                                         // store friendly orientation
    double x_original = pose.position.x;                                                                    // store friendly x original position
    double y_original = pose.position.y;                                                                    // sotre friendly original y position
    std::vector<RangeBearingStamped> bogies = friendly_scan;                                                // get the vector of the results from the friendly
    sort(bogies.begin(), bogies.end(), [](const RangeBearingStamped &lhs, const RangeBearingStamped &rhs) { // Sort targets based on passed data
        return lhs.range < rhs.range;
    });
    auto min_distance_relative_friendly = bogies.at(0);      // return the target based from the sorted passed vector target
    double distance = min_distance_relative_friendly.range;  // get the distance from the target
    bearing_angle_ = min_distance_relative_friendly.bearing; // get the bearing angle from the target
    angle_ = bearing_angle_ + friendly_orientation;          // adjust the bearing in reference to the friendly
    double final_x;                                          // place holder to final target position
    double final_y;                                          // place holder to final target position
    final_x = x_original + distance * cos(angle_);           // calculate final x position of target
    final_y = y_original + distance * sin(angle_);           // calculate final x psition target
    target_.push_back(std::make_pair(final_x, final_y));     // store final x and y positions of target
    bogies.clear();                                          // clear data
}
std::vector<std::pair<double, double>> Scanner::getTarget()
{
    return target_; // return vector of target position in x and y
}
std::vector<std::pair<double, double>> Scanner::getBaseScan()
{
    return base_scanner_; // return range and timestamp
}
std::vector<std::pair<double, double>> Scanner::getFriendlyScan()
{
    return friendly_scanner_; //return range and bearing from friendly
}
std::vector<std::pair<double, double>> Scanner::getBaseToFriendly()
{
    return friendly_relative_to_base_; // return friendly relative to base (tangetial, angle)
}
int Scanner::getBogieIndex()
{
    return bogie_index_; // return bogie target
}
void Scanner::resetData() // clear all data
{
    target_.clear();
    base_data_scan_.clear();
    friendly_scanner_.clear();
    friendly_relative_to_base_.clear();
    base_scanner_.clear();
    predicted_target_.clear();
    velocity_object_.clear();
    velocity_vector_.clear();
}

void Scanner::predictTarget(std::vector<RangeBearingStamped> &initial, std::vector<RangeBearingStamped> &secondary)
{
    double original_x, original_y, second_x, second_y, predicted_x, predicted_y; // positions place holders
    double timestamp_1, timestamp_2;                                             // times placeholders
    std::vector<std::pair<double, double>> original_position_ = getTarget();     // get the vector of position
    original_x = original_position_.front().first;                               // get the original x position
    original_y = original_position_.front().second;                              // get the original y position
    timestamp_1 = initial.front().timestamp;                                     // get initial timestamp
    std::this_thread::sleep_for(std::chrono::milliseconds(100));                 // wait to get data
    second_x = original_position_.back().first;                                  // get second position in place in x
    second_y = original_position_.back().second;                                 // get second position in y
    timestamp_2 = secondary.back().timestamp;                                    // get the second timestamp
    double time_def = (timestamp_2 - timestamp_1);                               // calculate the time difference between poses
    double velocity_x = (second_x - original_x) / time_def;                      // calculate velocity in x
    double velocity_y = (second_y - original_y) / time_def;                      // calculate velocity in y
    double vx = (second_x - original_x) / 0.11 + time_def;                       // calculate vector velocity in x with all the time difference
    double vy = (second_y - original_y) / 0.11 + time_def;                       // calculate vector velocity in y with all the time difference
    velocity_vector_.push_back(std::make_pair(vx, vy));                          // Store in vector vx and vy
    predicted_x = second_x + velocity_x * (time_def + 0.11);                     // get the predicted x position using the velocity vector
    predicted_y = second_y + velocity_y * (time_def + 0.11);                     // get the predicted y position using the velocity vector
    double velocity = sqrt((pow(vx, 2) + pow(vy, 2)));                           // get the scalar velocity
    velocity_object_.push_back(velocity);                                        // store the scalr velocity
    predicted_orientation_ = atan2(predicted_y, predicted_x);                    // get the predicted orientation
    if ((second_y < original_y) && (second_x < original_x))                      // consider position and adjust orientation
    {
        predicted_orientation_ = M_PI + fabs(predicted_orientation_);
    }
    else if ((second_y > original_y) && (second_x > original_x))
    {
        predicted_orientation_ = fabs(predicted_orientation_);
    }
    else if ((second_y > original_y) && (second_x < original_x))
    {
        predicted_orientation_ = (M_PI)-fabs(predicted_orientation_);
    }
    else if ((second_y < original_y) && (second_x > original_x))
    {
        predicted_orientation_ = (2 * M_PI) - fabs(predicted_orientation_);
    }
    else
    {
        predicted_orientation_ = bearing_angle_;
    }

    predicted_target_.push_back(std::make_pair(predicted_x, predicted_y)); // store predicted position in x and y
}
std::vector<std::pair<double, double>> Scanner::getPredictedTarget()
{
    return predicted_target_; // return vector of pairs of predicted target un x and y
}
std::vector<double> Scanner::getObjectVelocity()
{
    return velocity_object_; // return scalar velocity
}
double Scanner::getBearingAngle()
{
    return bearing_angle_; // return bearing angle of target
}
std::vector<std::pair<double, double>> Scanner::getVelocityVector()
{
    return velocity_vector_; // return velocity vector
}
double Scanner::getOrientationPrediction()
{
    return predicted_orientation_; // get predicted orientation
}
