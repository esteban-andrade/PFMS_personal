#include <vector>
#include <deque>
#include <condition_variable>
#include "simulator.h"
#include "datasynchronization.h"

static const int BUFFER = 10;

DataSynchronization::DataSynchronization()
{
}

DataSynchronization::~DataSynchronization()
{
}
void DataSynchronization::getBaseData(std::vector<RangeVelocityStamped> base_to_boggie)
{
    //base_data_.push_back(0.0);
    //cv_.notify_all();
    base_data_ = base_to_boggie;
    //cv_1_.notify_all();
}
void DataSynchronization::checkBaseSize()
{
    if (base_data_.size() == BUFFER)
    {
        base_data_.pop_back();
    }
}
void DataSynchronization::getFriendlyData(std::vector<RangeBearingStamped> friendly_to_boggie)
{
    friendly_data_ = friendly_to_boggie;
}
void DataSynchronization::checkFriendlySize(std::unique_lock<std::mutex> &locker)
{

    if (friendly_data_.size() == BUFFER)
    {
        RangeBearingStamped x_coordinate = friendly_data_.front();
        RangeBearingStamped y_coordinate = friendly_data_[1];
        for (int i = y_coordinate.timestamp; i <= x_coordinate.timestamp; i++)
        {
            timer_friendly_.push_back(i);
        }
        for (auto j : timer_friendly_)
        {
            friendly_range_.push_back(interpolate(y_coordinate.timestamp, x_coordinate.timestamp, y_coordinate.range, x_coordinate.range, j, true));
        }
        //cv_.wait(locker);
        // cv_2_.notify_all();
        friendly_data_.clear();
    }
}
void DataSynchronization::showData(std::unique_lock<std::mutex> &locker)
{
    //cv_1_.wait(locker);
    base_comparison_.push_back(std::make_pair(base_data_.front().timestamp, base_data_.front().range));
    //cv_2_.wait(locker);
    for (unsigned int i = 0; i < timer_friendly_.size() - 1; i++)
    {
        if (timer_friendly_[i] == base_comparison_.front().first)
        {
            friendly_comparison_.push_back(std::make_pair(timer_friendly_[i], friendly_range_[i]));
        }
    }
    timer_friendly_.clear();
    friendly_range_.clear();
}
std::vector<std::pair<long, long>> DataSynchronization::getBaseCompare()
{
    return base_comparison_;
}
std::vector<std::pair<long, long>> DataSynchronization::getFrindlyCompare()
{
    return friendly_comparison_;
}
void DataSynchronization::resetData()
{
    base_comparison_.clear();
    friendly_comparison_.clear();
}
double DataSynchronization::interpolate(long xL, long xR, double yL, double yR, double x, bool extrapolate)
{
    if (!extrapolate)
    {
        if (x < xL)
        {
            yR = yL;
        }
        if (x > xR)
        {
            yL = yR;
        }
        double dydx = (yR - yL) / (xR - xL);
        return yL + dydx * (x - xL);
    }
}
