#ifndef SCANOBJECTS_H
#define SCANOBJECTS_H

#include <cmath>
#include <vector>
#include "types.h"

class ScanObjects
{
private:
    std::vector<Pose> boggies_;
    std::vector<Pose> analysed_boggie_;
    std::vector<std::pair<double, double>> initial_position_;
    std::vector<std::pair<double, double>> second_position_;
    std::vector<std::pair<double, double>> distances_;

public:
    ScanObjects();
    ~ScanObjects();
    bool circleIntersection(GlobalOrd aircraft, GlobalOrd base, double distance_boggie_to_aircraft,
                            double distance_boggie_to_base, Pose &, Pose &);
    void findReferencePosition(Pose &, Pose &);
    void analyseBoggie(bool, Pose &, Pose &);
    void findCorrectedPosition(Pose &, Pose &);
    std::vector<Pose> getBoggies();
    std::vector<Pose> getAnalysedBoggies();
};

#endif //SCANOBJECTS