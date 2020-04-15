#include <iostream>
#include <vector>
#include <chrono>
#include <random>

#include "rectangle.h"
#include "triangle.h"
#include "shape.h"
#include "circle.h"

using std::cout;
using std::endl;
using std::vector;

//!TODO - TASK 4: Printing description - Completing this / think about auto

void printArea(vector<Shape*> shapes)
{
    for(auto s : shapes)
    {
        cout << s->getArea() << endl;
         cout << s->getDescription() << endl;
    }
    //shapes[0]->setHeightWidth(10, 10);
}

int main () {

    //!TODO - TASK 2: Create a Square and Traingle, and store both of them in a vector of type `Shape`
    Rectangle rectangle1;
    rectangle1.setHeightWidth(5, 5);
    Triangle triangle1(5, 5);
    Circle circle1(10);

    vector<Shape*> shapes;

    shapes.push_back(&rectangle1);
    shapes.push_back(&triangle1);
    shapes.push_back(&circle1);

    printArea(shapes);

    std::vector<int> hello;
    hello.push_back(1);
    hello.push_back(2);

    std::cout << hello.at(0);


    cout << "Please specify amount of circles wanted: ";
    int circle_amount;
    std::cin >> circle_amount;
    cout << "Please specify max radius: ";
    double max_radius;
    std::cin >> max_radius;
    cout << "Please specify max length: ";
    double max_length;
    std::cin >> max_length;
    double total_length=0;

    std::vector<Circle*> circles;

   Circle *circle_objects = new Matrix(circle_amount);

    for(int i = 0; i < circle_amount; i++)
    {
        unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<> value_distribution(0, max_length);
        Circle circle(value_distribution(generator));
        std::cout << value_distribution(generator) << std::endl;
        circles.push_back(&circle);
        std::cout << (circles.at(i))->getRadius() << std::endl;
        max_length -= ((circles.at(i))->getRadius())*2;
        total_length += ((circles.at(i))->getRadius())*2;
    }

    for (auto s : circles)
    {
        cout << "Radius is " << s->getRadius()*2 << std::endl;
    }
    std::cout << total_length << std::endl;




    //!TODO - TASK 5: Write a program that allows the user to specify number of circles and `max_radius`.
    //! Create the circles with random lengths to be capped to `max_length`.


}
