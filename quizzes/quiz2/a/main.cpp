#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <thread>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"
#include "shape.h"

using std::cout;
using std::endl;
using std::vector;

//!TODO - TASK 4: Printing description - Completing this / think about auto
void displayArea(vector<Shape *> &vec);

int main()
{

    //!TODO - TASK 2: Create a Square and Traingle, and store both of them in a vector of type `Shape`

    vector<Shape *> shapes;
    Rectangle *rectangle = new Rectangle;
    rectangle->setHeightWidth(5, 5);
    Triangle *triangle = new Triangle(5, 4);
    shapes.push_back(rectangle);
    shapes.push_back(triangle);
    displayArea(shapes);

    //!TODO - TASK 5: Write a program that allows the user to specify number of circles and `max_radius`.
    //! Create the circles with random lengths to be capped to `max_length`.
    int num_circles;
    double max_radius;
    std::cout << "Please enter number of circles to be generated: " << std::endl;
    while (!(std::cin >> num_circles) || num_circles < 0)
    {
        if (!(std::cin))
        {
            std::cout << "Invalid input, Try again: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        else if (num_circles < 0)
            std::cout << "Invalid input, Please enter a valid positive number : ";
    }

    std::cout << "Please specify max radius" << std::endl;

    while (!(std::cin >> max_radius) || max_radius < 0)
    {
        if (!(std::cin))
        {
            std::cout << "Invalid input, Try again: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        else if (max_radius < 0)
            std::cout << "Invalid input, Please enter a valid positive number : ";
    }

    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a uniform distribution between 0 and 10 and draw elements from it
    std::uniform_real_distribution<> value_distribution(0, max_radius);
    vector<Shape *> generatedShapes;
    for (int i = 0; i < num_circles; i++)
    {
        Circle *circle = new Circle(value_distribution(generator));
        generatedShapes.push_back(circle);
    }
    std::cout << "number of circles generated "<<generatedShapes.size() << std::endl;
    
    displayArea(generatedShapes);

    return 0;
}
void displayArea(vector<Shape *> &vec)
{
    for (auto shape : vec)
    {
        std::cout << "The area is :" << shape->getArea() << "\t"
                  << "The shapes is " << shape->getDescription() << std::endl;
    }
}