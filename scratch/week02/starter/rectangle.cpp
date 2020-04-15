#include "rectangle.h"

// Default Constructor
Rectangle::Rectangle() {}

// methods
void Rectangle::setWidthHeight(double width, double height)
{
    width_ = width;
    height_ = height;
}

double Rectangle::area()
{
    double area = width_ * height_;
    return area;
}

double Rectangle::perimeter()
{
    double perimeter = width_ + width_ + height_ + height_;
    return perimeter;
}