#include <cmath>
#include <vector>
#include "Seeker.h"

Seeker::Seeker()
{
}
Seeker::~Seeker()
{
}
bool Seeker::circleIntersection(GlobalOrd friendly, GlobalOrd base, double bogie_to_friendly, double bogie_to_base, Pose &bogie_0, Pose &bogie_1)
{
    double a, dx, dy, d, h, rx, ry, x2, y2;
    dx = friendly.x - base.x;
    dy = friendly.x - base.y;
    d = hypot(dx, dy);
    if (d > (bogie_to_friendly + bogie_to_base))
    {
        return false;
    }
    if (d < fabs(bogie_to_base - bogie_to_friendly))
    {
        return false;
    }
    a = ((bogie_to_base * bogie_to_base) - (bogie_to_friendly * bogie_to_friendly) + (d * d)) / (2.0 * d);
    x2 = base.x + (dx * a / d);
    y2 = base.y + (dy * a / d);
    h = sqrt((bogie_to_base * bogie_to_base) - (a * a));
    rx = -dy * (h / d);
    ry = dx * (h / d);
    bogie_0.position.x = x2 + rx;
    bogie_0.position.y = y2 + ry;
    bogie_1.position.x = x2 - rx;
    bogie_1.position.x = y2 - ry;
    return true;
}
void Seeker::findPosition(Pose &bogie_0, Pose &bogie_1)
{
    x_position_0_.push_back(bogie_0.position.x);
    y_position_0_.push_back(bogie_0.position.y);
    x_position_1_.push_back(bogie_1.position.x);
    y_position_1_.push_back(bogie_1.position.y);
    while (x_position_0_.size() == 2 && y_position_0_.size() == 2)
    {
        distance_0_ = sqrt(pow(x_position_0_.back() - x_position_0_.front(), 2) + pow(y_position_0_.back() - y_position_0_.front(), 2));
        distance_1_ = sqrt(pow(x_position_1_.back() - x_position_1_.front(), 2) + pow(y_position_1_.back() - y_position_1_.front(), 2));
        bogie_0.orientation = atan2(y_position_0_.back() - y_position_0_.front(), x_position_0_.back() - x_position_0_.front());
        bogie_1.orientation = atan2(y_position_1_.back() - y_position_1_.front(), x_position_1_.back() - x_position_1_.front());
        if (bogie_1.orientation <= 0)
        {
            bogie_1.orientation = 2 * M_PI - fabs(bogie_1.orientation);
        }
        if (bogie_0.orientation <= 0)
        {
            bogie_0.orientation = 2 * M_PI - fabs(bogie_0.orientation);
        }
        x_position_0_.clear();
        y_position_0_.clear();
        x_position_1_.clear();
        y_position_1_.clear();
    }
}
void Seeker::analyseBogie(bool start, Pose &bogie_0, Pose &bogie_1)
{
    if (start)
    {
        start = false;
        predicted_bogie_.push_back(bogie_0);
    }
    else
    {
        bogies_.push_back(bogie_0);
        bogies_.push_back(bogie_1);
    }
}
void Seeker::getPosition(Pose &bogie_0, Pose &bogie_1)
{
    if (bogies_.size() == 2)
    {
        double start_val = sqrt(pow(predicted_bogie_.front().position.x, 2) + pow(predicted_bogie_.front().position.x, 2));
        double start_orientation = predicted_bogie_.front().orientation;
        double offset_0 = fabs(start_orientation - bogie_0.orientation);
        double offset_1 = fabs(start_orientation - bogie_1.orientation);
        if (distance_0_ < distance_1_ && offset_0 < offset_1)
        {
            predicted_bogie_.push_back(bogie_0);
        }
        else if (distance_0_ > distance_1_ && offset_0 < offset_1)
        {
            predicted_bogie_.push_back(bogie_1);
        }
        else if (distance_0_ > distance_1_ && offset_0 > offset_1)
        {
            predicted_bogie_.push_back(bogie_1);
        }
        else if (distance_0_ < distance_1_ && offset_0 > offset_1)
        {
            predicted_bogie_.push_back(bogie_0);
        }
        bogies_.clear();
        predicted_bogie_.pop_back();
    }
}
std::vector<Pose> Seeker::getBogie()
{
    return bogies_;
}
std::vector<Pose> Seeker::getPredictedBogie()
{
    return predicted_bogie_;
}