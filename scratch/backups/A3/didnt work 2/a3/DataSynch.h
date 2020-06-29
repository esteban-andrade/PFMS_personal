#ifndef DATASYNCH_H
#define DATASYNCH_H
#include "simulator.h"
#include "types.h"
#include <vector>
#include <mutex>
#include <condition_variable>

class DataSynch
{
private:
    std::vector<double> time_friendly_;
    std::vector<double> range_friendly_;
    std::vector<RangeVelocityStamped> base_data_scan_;
    std::vector<RangeBearingStamped> friendly_data_scan_;
    std::condition_variable cv;
    std::condition_variable cv1;
    std::condition_variable cv2;
    std::vector<double> base_compare_;
    std::vector<double> friendly_compare_;

public:
    DataSynch();
    ~DataSynch();
    void acquireBaseData(std::vector<RangeVelocityStamped>);
    void checkBaseSize();
    void acquireFriendlyData(std::vector<RangeBearingStamped>);
    void checkFriendlySize(std::unique_lock<std::mutex> &);
    void showData(std::unique_lock<std::mutex> &);
    std::vector<double> getBaseCompare();
    std::vector<double> getFriendlyCompare();
    void resetData();
    double interpolateData(double, double, double, double, double, bool);
};

#endif //DATASYNCH_H