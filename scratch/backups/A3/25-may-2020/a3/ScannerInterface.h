#ifndef SCANNERINTERFACE_H
#define SCANNERINTERFACE_H
#include "simulator.h"
#include "types.h"

class ScannerInterface
{

public:
    ScannerInterface(){};
    ~ScannerInterface(){};
    virtual void resetData() = 0;
    virtual void determineTarget(Pose, std::vector<RangeBearingStamped>) = 0;
};

#endif //SCANNERINTERFACE_H
