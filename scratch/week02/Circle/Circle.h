#ifndef CIRCLE_H
#define CIRCLE_H

// Declaration of the rectangle class
class Circle
{
    // Public members are accessible from outside the class (ie. in main)
public:
    // Declare default the constructor
    Circle();
    // Declare constructor that sets values
    Circle(double radius);
    // Declare the set_values method
    void setRadius(double radius);
    // Declare the area method
    long double getArea();
    //Declare perimeter method
    long double getPerimeter();

    // Private members are only accessible from within methods of the same class
private:
    // This class has two integers to represent the sides of the rectangle
    // The trailing underscore is used to differentiate the member varibles
    // ..from local varibles in our code, this is not compulsary
    double radius_;
};

#endif // CIRCLE_H
