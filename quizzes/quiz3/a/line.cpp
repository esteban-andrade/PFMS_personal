#include "line.h"
#include <cmath>

Line::Line() : gradient_(0.0),
               y_intercept_(0.0)
{
}

Line::Line(double gradient, double y_intercept) : gradient_(gradient),
                                                  y_intercept_(y_intercept)
{
}

Line::Line(double ax, double ay, double bx, double by)
{
    ax_ = ax;
    ay_ = ay;
    bx_ = bx;
    by_ = by;
    fromPoints(ax, ay, bx, by);
}

void Line::fromPoints(double ax, double ay, double bx, double by)
{
    ax_ = ax;
    ay_ = ay;
    bx_ = bx;
    by_ = by;
    gradient_ = (by - ay) / (bx - ax);
    y_intercept_ = ay - gradient_ * ax;
}
double Line::get_dr()
{
    double dr = 0;
    double dx = bx_ - ax_;
    double dy = by_ - ay_;
    dr = pow(dx, 2) + pow(dy, 2);
    return sqrt(dr);
}
double Line::getD()
{
    double D = 0;
    D = ax_ * by_ - bx_ * ay_;
    return D;
}
void Line::setGradient(double gradient)
{
    gradient_ = gradient;
}

void Line::setYIntercept(double y_intercept)
{
    y_intercept_ = y_intercept;
}

bool Line::pointAboveLine(double x, double y)
{
    double line_y = gradient_ * x + y_intercept_;
    return y > line_y;
}
