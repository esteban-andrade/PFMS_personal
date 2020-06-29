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
void Scanner::baseScan(const std::shared_ptr<Simulator> &sim)
{
    base_data_scan_ = sim->rangeVelocityToBogiesFromBase();
    for (unsigned int i = 0; i < base_data_scan_.size(); i++)
    {
        base_scanner_.push_back(std::make_pair(base_data_scan_.at(i).range, base_data_scan_.at(i).timestamp));
    }
}
void Scanner::airCraftScan(const std::shared_ptr<Simulator> &sim)
{

    friendly_data_scan_ = sim->rangeBearingToBogiesFromFriendly();
    for (unsigned int i = 0; i < friendly_data_scan_.size(); i++)
    {
        friendly_scanner_.push_back(std::make_pair(friendly_data_scan_.at(i).range, friendly_data_scan_.at(i).bearing));
    }
}
void Scanner::friendlyRelariveBase(const std::shared_ptr<Simulator> &sim)
{
    Pose Pose = sim->getFriendlyPose();
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
void Scanner::determineTarget(const std::shared_ptr<Simulator> &sim)
{
    auto min_distance_relative_friendly = std::min_element(friendly_scanner_.begin(), friendly_scanner_.end());
    auto index_min_distance_reading = std::min_element(friendly_scanner_.begin(), friendly_scanner_.end()) - friendly_scanner_.begin();
    bogie_index_ = index_min_distance_reading;
    double distance = min_distance_relative_friendly->first;
    double angle = min_distance_relative_friendly->second;
    Pose pose = sim->getFriendlyPose();
    double x_original = pose.position.x;
    double y_original = pose.position.y;
    double final_x;
    double final_y;

    final_x = x_original + distance * cos(angle);
    final_y = y_original + distance * sin(angle);
    target_.push_back(std::make_pair(final_x, final_y));
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
}

void Scanner::predictTarget(const std::shared_ptr<Simulator> &sim)
{
    double original_x, original_y, second_x, second_y, predicted_x, predicted_y;
    double timestamp_1, timestamp_2;
    std::vector<std::pair<double, double>> original_position_ = getTarget();
    original_x = original_position_.front().first;
    original_y = original_position_.front().second;
    timestamp_1 = sim->rangeBearingToBogiesFromFriendly().front().timestamp;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    second_x = original_position_.back().first;
    second_y = original_position_.back().second;
    timestamp_2 = sim->rangeBearingToBogiesFromFriendly().back().timestamp;
    double time_def = (timestamp_2 - timestamp_1) * 1000;
    double velocity_x = (second_x - original_x) / time_def;
    double velocity_y = (second_y - original_y) / time_def;
    predicted_x = original_x + velocity_x * time_def;
    predicted_y = original_y + velocity_y * time_def;
    velocity_object_ = sqrt(pow(velocity_x, 2) + pow(velocity_y, 2));
    predicted_target_.push_back(std::make_pair(predicted_x, predicted_y));
}
std::vector<std::pair<double, double>> Scanner::getPredictedTarget()
{
    return predicted_target_;
}
double Scanner::getObjectVelocity()
{
    return velocity_object_;
}
