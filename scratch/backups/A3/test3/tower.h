#ifndef TOWER
#define TOWER
#include "simulator.h"

class Tower{
public:
    Tower(){};
    virtual Pose getCurrentPose() = 0;
    virtual double getLinSpeed() = 0;
    virtual void setSimulator(const std::shared_ptr<Simulator> & sm) = 0;
private:
};
#endif // TOWER

