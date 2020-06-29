#ifndef FRIEND
#define FRIEND
#include "tower.h"

class Friendly: public Tower{
public:
    Friendly();

    void setSimulator(const std::shared_ptr<Simulator> & sm);
    Pose getCurrentPose();
    void setCurrentPose();
    bool inAirspace();
    double getLinSpeed();
    void setLinSpeed();
    double getAngSpeed();
    void setAngSpeed();
    void checkBoundary();
    void destroy(const RangeBearingStamped &);
private:
    std::shared_ptr<Simulator> sim_;
    Pose current_pose_;
    double lin_velocity_, ang_velocity_;
    bool inside_airspace_;
};
#endif // FRIEND

