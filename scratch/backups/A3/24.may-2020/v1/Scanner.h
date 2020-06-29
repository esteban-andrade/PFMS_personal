#ifndef SCANNER_H
#define SCANNER_H

#include "simulator.h"
#include "types.h"
#include <cmath>
#include <vector>
#include "ScannerInterface.h"

class Scanner : public ScannerInterface
{
private:
    std::vector<std::pair<double, double>> base_scanner_;              //m and t
    std::vector<std::pair<double, double>> friendly_scanner_;          // m and theta
    std::vector<std::pair<double, double>> friendly_relative_to_base_; // m and theta
    std::vector<RangeVelocityStamped> base_data_scan_;
    std::vector<RangeBearingStamped> friendly_data_scan_;
    std::vector<std::pair<double, double>> target_; //x an y
    int bogie_index_;
    std::vector<std::pair<double, double>> predicted_target_;
    std::vector<double> velocity_object_;
    double bearing_angle_;
    double angle_;
    double range_;
    double predicted_orientation_;
    std::vector<std::pair<double, double>> velocity_vector_;
    Pose current_pose_;

public:
    Scanner();
    ~Scanner();
    void determineTarget(const std::shared_ptr<Simulator> &);
    void friendlyRelariveBase(const std::shared_ptr<Simulator> &);
    void baseScan(const std::shared_ptr<Simulator> &);
    void airCraftScan(const std::shared_ptr<Simulator> &);
    std::vector<RangeVelocityStamped> getBaseScanResults();
    std::vector<RangeBearingStamped> getFriendlyScanResults();
    std::vector<std::pair<double, double>> getTarget();
    void resetData();
    std::vector<std::pair<double, double>> getBaseScan();
    std::vector<std::pair<double, double>> getFriendlyScan();
    std::vector<std::pair<double, double>> getBaseToFriendly();
    int getBogieIndex();
    void predictTarget(const std::shared_ptr<Simulator> &);
    std::vector<double> getObjectVelocity();
    std::vector<std::pair<double, double>> getPredictedTarget();
    double getBearingAngle();
    std::vector<std::pair<double, double>> getVelocityVector();
    double getOrientationPrediction();
};

#endif //SCANNER_H