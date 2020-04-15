#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape
{
public:
    Rectangle();
    void setHeightWidth(double width, double height);
    
    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Consider if getArea() is a method that should exist in Rectangle?
    // * this will depend whether we need to retrieve the area of multiple shapes, If so we need to implement it to the parent class
    // Should all shapes be able to computer Area? Do all shapes have this attribute?
    // * all common shapes such as circles, squares, triangles and many other paralelograms should be fine. Otherwise other forms need to be considered 
    // A design to enable this is covered in when we introduce polymorphism
    double getArea (void);
private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?

    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
