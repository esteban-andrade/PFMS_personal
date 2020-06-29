#ifndef DATASYNCHRONIZATION_H
#define DATASYNCHRONIZATION_H

#include <vector>
#include <deque>
#include <condition_variable>
#include "simulator.h"

class DataSynchronization
{
private:
    std::vector<double> timer_friendly_;
    std::vector<double> friendly_range_;
    std::vector<RangeVelocityStamped> base_data_;
    std::vector<RangeBearingStamped> friendly_data_;
    std::condition_variable cv_;
    std::condition_variable cv_1_;
    std::condition_variable cv_2_;
    std::vector<std::pair<long, long>> base_comparison_;
    std::vector<std::pair<long, long>> friendly_comparison_;

public:
    DataSynchronization();
    ~DataSynchronization();
    void getBaseData(std::vector<RangeVelocityStamped>);
    void checkBaseSize();
    void getFriendlyData(std::vector<RangeBearingStamped>);
    void checkFriendlySize(std::unique_lock<std::mutex> &);
    void showData(std::unique_lock<std::mutex> &);
    std::vector<std::pair<long, long>> getBaseCompare();
    std::vector<std::pair<long, long>> getFrindlyCompare();
    void resetData();
    double interpolate(long, long, double, double, double, bool);
};

#endif //DATASYNCHRONIZATION_H