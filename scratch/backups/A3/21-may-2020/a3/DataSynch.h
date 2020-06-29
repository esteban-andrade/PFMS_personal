#ifndef DATASYNCH_
#define DATASYNCH_

#include <vector>
#include <mutex>
#include <condition_variable>
#include "Navigation.h"
#include "Scanner.h"
#include "simulator.h"
#include "types.h"

class DataSynch
{
private:
    std::mutex mtx;
    std::condition_variable cv;
    double bearing_target_;
    double velocity_x_;
    double velocity_y_;
    RangeBearingStamped closest_bogie_;
    std::vector<RangeBearingStamped> friendly_scan_results_;
    bool process_ready_;
    bool target_acquired_;

public:
    DataSynch();
    ~DataSynch();
    void dataThread(Navigation &, const std::shared_ptr<Simulator> &, Pose);
    void controlThread(const std::shared_ptr<Simulator> &, Navigation &, Scanner &, DataSynch &);
    void processThread(Scanner &);
    // void setNearestBogieRange(Scanner &);
    // GlobalOrd calculateCurrentPose(Navigation &);
    // void calculateVelocity(Navigation &);
    // void predictPosition(Navigation &);
};

#endif //DATASYNCH_