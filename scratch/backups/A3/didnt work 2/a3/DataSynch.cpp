#include <vector>
#include <mutex>
#include <condition_variable>
#include "simulator.h"
#include "types.h"
#include "DataSynch.h"

static const int BUFFER = 10;

DataSynch::DataSynch()
{
}
DataSynch::~DataSynch()
{
}
void DataSynch::acquireBaseData(std::vector<RangeVelocityStamped> base_to_bogie)
{
    cv.notify_all();
    base_data_scan_.push_back({0});
    base_data_scan_ = base_to_bogie;
    cv1.notify_all();
}
void DataSynch::checkBaseSize()
{
    if (base_data_scan_.size() == BUFFER)
    {
        base_data_scan_.pop_back();
    }
}
void DataSynch::acquireFriendlyData(std::vector<RangeBearingStamped> friendly_to_bogie)
{
    friendly_data_scan_.push_back({0});
    friendly_data_scan_ = friendly_to_bogie;
}
void DataSynch::checkFriendlySize(std::unique_lock<std::mutex> &locker)
{
    if (friendly_data_scan_.size() == BUFFER)
    {
        RangeBearingStamped x = friendly_data_scan_.back();
        RangeBearingStamped y = friendly_data_scan_.at(1);
        for (int i = y.timestamp; i <= x.timestamp; i++)
        {
            time_friendly_.push_back(i);
        }
        for (auto j : time_friendly_)
        {
            range_friendly_.push_back(interpolateData(y.timestamp, x.timestamp, y.range, x.range, j, true));
        }
        cv.wait(locker);
        cv2.notify_all();
        friendly_data_scan_.clear();
    }
}
void DataSynch::showData(std::unique_lock<std::mutex> &locker)
{
    cv1.wait(locker);
    base_compare_.push_back(base_data_scan_.front().timestamp);
    base_compare_.push_back(base_data_scan_.front().range);
    cv2.wait(locker);
    for (int i = 0; i <= time_friendly_.size() - 1; i++)
    {
        if (time_friendly_.at(i) == base_compare_.at(0))
        {
            friendly_compare_.push_back(time_friendly_.at(i));
            friendly_compare_.push_back(range_friendly_.at(i));
        }
    }
    time_friendly_.clear();
    range_friendly_.clear();
}
std::vector<double> DataSynch::getBaseCompare()
{
    return base_compare_;
}
std::vector<double> DataSynch::getFriendlyCompare()
{
    return friendly_compare_;
}
void DataSynch::resetData()
{
}
double DataSynch::interpolateData(double xL, double xR, double yL, double yR, double x, bool extrapolate)
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
    }
    double dydx = (yR - yL) / (xR - xL);
    return (yL + dydx * (x - xL));
}