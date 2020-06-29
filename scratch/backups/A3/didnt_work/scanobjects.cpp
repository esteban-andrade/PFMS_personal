#include <math.h>
#include <vector>
#include <cmath>
#include "scanobjects.h"

ScanObjects::ScanObjects()
{
}

ScanObjects::~ScanObjects()
{
}

bool ScanObjects::circleIntersection(GlobalOrd aircraft, GlobalOrd base, double distance_boggie_to_aircraft,
                                     double distance_boggie_to_base, Pose &boggie_1, Pose &boggie_2)
{
    double a, dx, dy, d, h, rx, ry, x2, y2;
    dx = aircraft.x - base.x;
    dy = aircraft.y - base.y;
    d = hypot(dx, dy);
    if (d > (distance_boggie_to_aircraft + distance_boggie_to_base))
    {
        return false;
    }
    if (d < fabs(distance_boggie_to_base - distance_boggie_to_aircraft))
    {
        return false;
    }
    a = ((pow(distance_boggie_to_base, 2)) - (pow(distance_boggie_to_aircraft, 2)) + (pow(d, 2))) / (d * 2.0);
    x2 = base.x + (dx * a / d);
    y2 = base.y + (dy * a / d);
    h = sqrt(pow(distance_boggie_to_base, 2) - pow(a, 2));
    rx = -dy * (h / d);
    ry = dx * (h / d);
    boggie_1.position.x = x2 + rx;
    boggie_1.position.y = y2 + ry;
    boggie_2.position.x = x2 - rx;
    boggie_2.position.y = y2 - ry;
    return true;
}
void ScanObjects::findReferencePosition(Pose &boggie_1, Pose &boggie_2)
{
    initial_position_.push_back(std::make_pair(boggie_1.position.x, boggie_1.position.y));
    second_position_.push_back(std::make_pair(boggie_2.position.x, boggie_2.position.y));
    while (initial_position_.size() == 1)
    {
        double distance_initial = sqrt(pow(initial_position_.front().first - initial_position_.back().first, 2) + pow(initial_position_.front().second - initial_position_.back().second, 2));
        double distance_subsequent = sqrt(pow(second_position_.front().first - second_position_.back().first, 2) + pow(second_position_.front().second - second_position_.back().second, 2));
        distances_.push_back(std::make_pair(distance_initial, distance_subsequent));
        boggie_1.orientation = atan2(initial_position_.front().second - initial_position_.back().second, initial_position_.front().first - initial_position_.back().first);
        boggie_2.orientation = atan2(second_position_.front().second - second_position_.back().second, second_position_.front().first - second_position_.back().first);
        if (boggie_1.orientation <= 0)
        {
            boggie_1.orientation = 2 * M_PI * fabs(boggie_1.orientation);
        }
        if (boggie_2.orientation <= 0)
        {
            boggie_2.orientation = 2 * M_PI * fabs(boggie_2.orientation);
        }
        initial_position_.clear();
        second_position_.clear();
    }
}

void ScanObjects::analyseBoggie(bool start, Pose &boggie_1, Pose &boggie_2)
{
    if (start)
    {
        start = false;
        analysed_boggie_.push_back(boggie_1);
    }
    else
    {
        boggies_.push_back(boggie_1);
        boggies_.push_back(boggie_2);
    }
}

void ScanObjects::findCorrectedPosition(Pose &boggie_1, Pose &boggie_2)
{
    std::vector<std::pair<double, double>> offsets;
    if (boggies_.size() == 2)
    {
        double initial_orientation = analysed_boggie_.front().orientation;

        double offset_1 = fabs(initial_orientation - boggie_1.orientation);
        double offset_2 = fabs(initial_orientation - boggie_2.orientation);
        offsets.push_back(std::make_pair(offset_1, offset_2));
        if (distances_.front().first < distances_.front().second && offsets.front().first < offsets.front().second)
        {
            analysed_boggie_.push_back(boggie_1);
        }
        else if (distances_.front().first > distances_.front().second && offsets.front().first < offsets.front().second)
        {
            analysed_boggie_.push_back(boggie_2);
        }
        else if (distances_.front().first > distances_.front().second && offsets.front().first > offsets.front().second)
        {
            analysed_boggie_.push_back(boggie_2);
        }
        else if (distances_.front().first < distances_.front().second && offsets.front().first > offsets.front().second)
        {
            analysed_boggie_.push_back(boggie_1);
        }
        boggies_.clear();
        analysed_boggie_.pop_back();
    }
}

std::vector<Pose> ScanObjects::getBoggies()
{
    return boggies_;
}

std::vector<Pose> ScanObjects::getAnalysedBoggies()
{
    return analysed_boggie_;
}
