#ifndef NAVIGATIONINTERFACE_H
#define NAVIGATIONINTERFACE_H

#include "simulator.h"
#include "types.h"

class NavigationInterface
{
public:
    NavigationInterface(){};
    ~NavigationInterface(){};
    virtual double getAngularSpeed() = 0;
    virtual double getLinearVelocity() = 0;
    virtual double getOrientation(Pose, Pose) = 0;
};

#endif //NAVIGATIONINTERFACE_H