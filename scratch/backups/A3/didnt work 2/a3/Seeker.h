#ifndef SEEKER_H
#define SEEKER_H
#include "simulator.h"
#include "types.h"
#include <cmath>
#include <vector>

class Seeker
{
private:
    std::vector<Pose> bogies_;
    std::vector<Pose> predicted_bogie_;
    double distance_0_;
    double distance_1_;
    std::vector<double> x_position_0_;
    std::vector<double> y_position_0_;
    std::vector<double> x_position_1_;
    std::vector<double> y_position_1_;

public:
    Seeker();
    ~Seeker();
    bool circleIntersection(GlobalOrd, GlobalOrd, double, double, Pose &, Pose &);
    void findPosition(Pose &, Pose &);
    void analyseBogie(bool, Pose &, Pose &);
    void getPosition(Pose &, Pose &);
    std::vector<Pose> getBogie();
    std::vector<Pose> getPredictedBogie();
};
#endif //SEEKER_H