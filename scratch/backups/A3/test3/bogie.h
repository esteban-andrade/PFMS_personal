#ifndef BOGIE
#define BOGIE
#include "tower.h"

class Bogie: public Tower{
public:
    Bogie();
    
    void setSimulator(const std::shared_ptr<Simulator> & sm);

    std::vector<RangeVelocityStamped> getBogiesFromBase();
    void setBogiesFromBase();

    std::vector<RangeBearingStamped> getBogiesFromFriend();
    void setBogiesFromFriend();

    Pose getCurrentPose();
    
    double getLinSpeed();
    double getAngSpeed();
protected:
    void setCurrentPose();
private:
    std::shared_ptr<Simulator> sim_;
    std::vector<RangeVelocityStamped> bogies_from_base_;
    std::vector<RangeBearingStamped> bogies_from_friend_;
    Pose current_pose_;
    double lin_velocity_, ang_velocity_;
    Aircraft targets;
};
#endif // BOGIE

