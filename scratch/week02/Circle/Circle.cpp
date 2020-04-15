#define _USE_MATH_DEFINES

#include "Circle.h"
#include <cmath>

// Default Constructor
Circle::Circle() {}
//Constructor that will set the radius
Circle::Circle(double radius) : radius_(radius) {}
// methods
void Circle::setRadius(double radius)
{
    radius_ = radius;
}

long double Circle::getArea()
{
    double area = M_PI * pow(radius_, 2);
    return area;
}

long double Circle::getPerimeter()
{
    double perimeter = 2.0 * M_PI * radius_;
    return perimeter;
}