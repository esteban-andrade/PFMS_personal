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
    base_data_scan_ = base_scan_sim;
    for (unsigned int i = 0; i < base_data_scan_.size(); i++)
    {
        base_scanner_.push_back(std::make_pair(base_data_scan_.at(i).range, base_data_scan_.at(i).timestamp));
    }
}
void Scanner::airCraftScan(std::vector<RangeBearingStamped> &raw_scan_sim)
{
    std::vector<RangeBearingStamped> raw_scan = raw_scan_sim;
    sort(raw_scan.begin(), raw_scan.end(), [](const RangeBearingStamped &lhs, const RangeBearingStamped &rhs) {
        return lhs.range < rhs.range;
    });

    friendly_data_scan_ = raw_scan;
    for (unsigned int i = 0; i < friendly_data_scan_.size(); i++)
    {
        friendly_scanner_.push_back(std::make_pair(friendly_data_scan_.at(i).range, friendly_data_scan_.at(i).bearing));
    }
}
void Scanner::friendlyRelativeBase(Pose &friendly_to_base)
{
    Pose Pose = friendly_to_base;
    double distance = sqrt(pow(Pose.position.x, 2) + pow(Pose.position.y, 2));
    double angle = Pose.orientation;
    friendly_relative_to_base_.push_back(std::make_pair(distance, angle));
}
std::vector<RangeVelocityStamped> Scanner::getBaseScanResults()
{
    return base_data_scan_;
}
std::vector<RangeBearingStamped> Scanner::getFriendlyScanResults()
{
    return friendly_data_scan_;
}
void Scanner::determineTarget(Pose pose_sim, std::vector<RangeBearingStamped> friendly_scan)
{

    Pose pose = pose_sim;
    double friendly_orientation = pose.orientation;
    double x_original = pose.position.x;
    double y_original = pose.position.y;
    std::vector<RangeBearingStamped> bogies = friendly_scan;
    sort(bogies.begin(), bogies.end(), [](const RangeBearingStamped &lhs, const RangeBearingStamped &rhs) {
        return lhs.range < rhs.range;
    });
    auto min_distance_relative_friendly = bogies.at(0);
    double distance = min_distance_relative_friendly.range;
    bearing_angle_ = min_distance_relative_friendly.bearing;
    angle_ = bearing_angle_ + friendly_orientation;
    double final_x;
    double final_y;

    final_x = x_original + distance * cos(angle_);
    final_y = y_original + distance * sin(angle_);
    target_.push_back(std::make_pair(final_x, final_y));
    bogies.clear();
}
std::vector<std::pair<double, double>> Scanner::getTarget()
{
    return target_;
}
std::vector<std::pair<double, double>> Scanner::getBaseScan()
{
    return base_scanner_;
}
std::vector<std::pair<double, double>> Scanner::getFriendlyScan()
{
    return friendly_scanner_;
}
std::vector<std::pair<double, double>> Scanner::getBaseToFriendly()
{
    return friendly_relative_to_base_;
}
int Scanner::getBogieIndex()
{
    return bogie_index_;
}
void Scanner::resetData()
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
    double original_x, original_y, second_x, second_y, predicted_x, predicted_y;
    double timestamp_1, timestamp_2;
    std::vector<std::pair<double, double>> original_position_ = getTarget();
    original_x = original_position_.front().first;
    original_y = original_position_.front().second;
    timestamp_1 = initial.front().timestamp;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    second_x = original_position_.back().first;
    second_y = original_position_.back().second;
    timestamp_2 = secondary.back().timestamp;
    double time_def = (timestamp_2 - timestamp_1);
    double velocity_x = (second_x - original_x) / time_def;
    double velocity_y = (second_y - original_y) / time_def;
    double vx = (second_x - original_x) / 0.11 + time_def;
    double vy = (second_y - original_y) / 0.11 + time_def;
    velocity_vector_.push_back(std::make_pair(vx, vy));
    predicted_x = second_x + velocity_x * (time_def + 0.11);
    predicted_y = second_y + velocity_y * (time_def + 0.11);
    double velocity = sqrt((pow(vx, 2) + pow(vy, 2)));
    velocity_object_.push_back(velocity);
    double predicted_orientation = atan2(predicted_y, predicted_x);
    // if ((predicted_y < original_y) && (predicted_x < original_x))
    // {
    //     predicted_orientation_ = M_PI + fabs(predicted_orientation);
    // }
    // else if ((predicted_y > original_y) && (predicted_x > original_x))
    // {
    //     predicted_orientation_ = fabs(predicted_orientation);
    // }
    // else if ((predicted_y > original_y) && (predicted_x < original_x))
    // {
    //     predicted_orientation_ = (M_PI)-fabs(predicted_orientation);
    // }
    // else if ((predicted_y < original_y) && (predicted_x > original_x))
    // {
    //     predicted_orientation_ = (2 * M_PI) - fabs(predicted_orientation);
    // }
    // else
    // {
    //     predicted_orientation_ = bearing_angle_;
    // }
    if ((second_y < original_y) && (second_x < original_x))
    {
        predicted_orientation_ = M_PI + fabs(predicted_orientation);
    }
    else if ((second_y > original_y) && (second_x > original_x))
    {
        predicted_orientation_ = fabs(predicted_orientation);
    }
    else if ((second_y > original_y) && (second_x < original_x))
    {
        predicted_orientation_ = (M_PI)-fabs(predicted_orientation);
    }
    else if ((second_y < original_y) && (second_x > original_x))
    {
        predicted_orientation_ = (2 * M_PI) - fabs(predicted_orientation);
    }
    else
    {
        predicted_orientation_ = bearing_angle_;
    }

    predicted_target_.push_back(std::make_pair(predicted_x, predicted_y));
}
std::vector<std::pair<double, double>> Scanner::getPredictedTarget()
{
    return predicted_target_;
}
std::vector<double> Scanner::getObjectVelocity()
{
    return velocity_object_;
}
double Scanner::getBearingAngle()
{
    return bearing_angle_;
}
std::vector<std::pair<double, double>> Scanner::getVelocityVector()
{
    return velocity_vector_;
}
double Scanner::getOrientationPrediction()
{
    return predicted_orientation_;
}
