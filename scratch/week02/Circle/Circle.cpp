#define _USE_MATH_DEFINES

#include "Circle.h"
#include <cmath>
#include <random>
#include <chrono>

// Default Constructor
Circle::Circle()
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    // std::random_device raw_data_seed;
}
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
double Circle::print()
{
    std::random_device raw_data_seed;
    

    std::default_random_engine generator(raw_data_seed());
    std::normal_distribution<double> distribution(4.0, 5.0);

    double test = distribution(generator);
    return test;
}