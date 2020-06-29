#ifndef INTERCEPT_BOGIE_H
#define INTERCEPT_BOGIE_H

#include "friend.h"
#include "bogie.h"
#include <mutex>
#include <condition_variable>

class InterceptBogies{
public:
    InterceptBogies(const std::shared_ptr<Simulator> & );

    //For example purposes only, this thread attmps to get the friendly aircraft's
    //(red triangle) pose every 4 seconds. It plots this pose on the
    //simulation (blue triangle) which stays on the image for 1 second, as per the
    //'testPose()' documentation in the simualtor class.
    void dataThread();

    //This thread will simply get the current velocity and feed it back into
    //controlled velocity, at the designated minimum time required (watchdog time) refer
    //'controlFriendly()' documentation in the simualtor class.
    void controlThread();

    void processThread();
private:
    // void checkBoundary();
    void setNearestBogieRange();
    GlobalOrd calculateCurrentPose();
    void calculateVelocity();
    void predictPos();

    std::mutex mu;
    std::condition_variable cv;
    double bogie_bearing_;
    std::shared_ptr<Simulator> sim_;
    Friendly f1_;
    Bogie b1_;
    double velocity_x_;
    double velocity_y_;

    RangeBearingStamped nearest_bogie_;
    std::vector<RangeBearingStamped> bogie_from_base_;
    bool process_ready_;
    bool target_acquired_;
};
#endif // INTERCEPT_BOGIE_H
