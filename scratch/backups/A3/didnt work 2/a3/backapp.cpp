#include "simulator.h"
#include "types.h"
#include <cmath>
#include <vector>
#include "Scanner.h"

Scanner::Scanner()
{
}
Scanner::~Scanner()
{
}
bool Scanner::circleIntersection(const std::shared_ptr<Simulator> &sim, GlobalOrd friendly, GlobalOrd base, double bogie_to_friendly_range, double bogie_to_base_range, Pose &bogie_0, Pose &bogie_1)
{
    double a, dx, dy, d, h, rx, ry;
    double x2, y2;
    dx = friendly.x - base.x;
    dy = friendly.y - base.y;

    d = hypot(dx, dy);
    if (d > (bogie_to_friendly_range + bogie_to_friendly_range))
    {
        return false;
    }
    if (d < fabs(bogie_to_base_range - bogie_to_friendly_range))
    {
        return false;
    }
    a = ((bogie_to_base_range * bogie_to_base_range) - (bogie_to_friendly_range * bogie_to_friendly_range) + (d * d)) / (d * 2.0);
    x2 = base.x + (dx * a / d);
    y2 = base.y + (dy * a / d);
    h = sqrt((bogie_to_base_range * bogie_to_base_range) - (a * a));
    rx = -dy * (h / d);
    ry = dx * (h / d);
    bogie_0.position.x = x2 + rx;
    bogie_0.position.y = y2 + ry;
    bogie_1.position.x = x2 - rx;
    bogie_1.position.y = y2 - ry;
    return true;
}
double Scanner::getDistance(const std::shared_ptr<Simulator> &sim, GlobalOrd point_1, GlobalOrd point_2)
{
    double dist = sim->distance(point_1, point_2);
    return dist;
}
void Scanner::findPosition(Pose &bogie_0, Pose &bogie_1)
{
    //initial_position_.push_back(std::make_pair(bogie_0.position.x, bogie_0.position.y));
    //second_position.push_back(std::make_pair(bogie_1.position.x, bogie_1.position.y));
    x_position_0.push_back(bogie_0.position.x);
    y_position_0.push_back(bogie_0.position.y);
    x_position_1.push_back(bogie_1.position.x);
    y_position_1.push_back(bogie_1.position.y);

    while (x_position_0.size() == 2 && y_position_0.size() == 2)
    {
        distance_1 = sqrt(pow(x_position_0.back() - x_position_0.front(), 2) + pow(y_position_0.back() - y_position_0.front(), 2));
        distance_2 = sqrt(pow(x_position_1.back() - x_position_1.front(), 2) + pow(y_position_1.back() - y_position_1.front(), 2));
        bogie_0.orientation = atan2(y_position_0.back() - y_position_0.front(), x_position_0.back() - x_position_0.front());
        bogie_1.orientation = atan2(y_position_1.back() - y_position_1.front(), x_position_1.back() - x_position_1.front());
        if (bogie_1.orientation <= 0)
        {
            bogie_1.orientation = 2 * M_PI - fabs(bogie_1.orientation);
        }
        if (bogie_0.orientation <= 0)
        {
            bogie_0.orientation = 2 * M_PI - fabs(bogie_0.orientation);
        }
        x_position_0.clear();
        y_position_0.clear();
        x_position_1.clear();
        y_position_1.clear();
    }
}
void Scanner::trackBogie(bool start, Pose &bogie_0, Pose &bogie_1)
{
    if (start)
    {
        start = false;
        predicted_bogie_position_.push_back(bogie_0);
    }
    else
    {
        bogie_position_.push_back(bogie_0);
        bogie_position_.push_back(bogie_1);
    }
}
void Scanner::predictPosition(Pose &bogie_0, Pose &bogie_1)
{
    if (bogie_position_.size() == 2)
    {
        double start_value = sqrt(pow(predicted_bogie_position_.back().position.x, 2) + pow(predicted_bogie_position_.back().position.x, 2));
        double start_orientation = predicted_bogie_position_.back().orientation;
        double offset_1 = fabs(start_orientation - bogie_0.orientation);
        double offset_2 = fabs(start_orientation - bogie_1.orientation);
        if (distance_1 < distance_2 && offset_1 < offset_2)
        {
            predicted_bogie_position_.push_back(bogie_0);
        }
        else if (distance_1 > distance_2 && offset_1 < offset_2)
        {
            predicted_bogie_position_.push_back(bogie_1);
        }
        else if (distance_1 > distance_2 && offset_1 > offset_2)
        {
            predicted_bogie_position_.push_back(bogie_1);
        }
        else if (distance_1 < distance_2 && offset_1 > offset_2)
        {
            predicted_bogie_position_.push_back(bogie_0);
        }
        bogie_position_.clear();
        predicted_bogie_position_.pop_back();
    }
}
std::vector<Pose> Scanner::getBogie()
{
    return bogie_position_;
}
std::vector<Pose> Scanner::getPredictedBogie()
{
    return predicted_bogie_position_;
}