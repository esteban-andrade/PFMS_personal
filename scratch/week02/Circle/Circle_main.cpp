// We need to include the declaration of our new rectangle class in order to use it.
#include "Circle.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <random>
#include <chrono>

//function Declation
double computeTotalArea(std::vector<Circle> circles);
double computeTotalPerimeter(std::vector<Circle> circles);

int main()
{

    {
        double rad = 1.0;

        // Create a rectangle object
        Circle circle;
        std::cout << circle.print() << std::endl;
        std::cout << circle.print() << std::endl;
        std::cout << circle.print() << std::endl;
        std::cout << circle.print() << std::endl;

        // Set the values of the sides
        std::cout << " The Circle test Radius is: " << rad << std::endl;
        circle.setRadius(rad);

        // set the to display output to have two decimal points
        std::cout << std::fixed << std::setprecision(2);

        // Get the area and print it to screen
        std::cout << " The Circle test Area is: " << circle.getArea() << std::endl;

        //Get the Perimeter and Print it
        std::cout << " The Circle test Perimeter is: " << circle.getPerimeter() << std::endl;
    }
    std::cout << "\nThis is Q2" << std::endl;
    Circle circle_1(1.0);
    Circle circle_2(2.0);
    Circle circle_3(5.0);
    std::cout << "There are 3 circles and the combined area is: " << circle_1.getArea() + circle_2.getArea() + circle_3.getArea() << "\n"
              << "The combined perimeter is: " << circle_1.getPerimeter() + circle_2.getPerimeter() + circle_3.getPerimeter() << "\n"
              << std::endl;

    std::cout << "This is Q3" << std::endl;
    // way we can store multiple objects of the same class in a container

    std::vector<Circle> circles;

    // this is  the traditional way of creating objects
    // circleVec.push_back(Circle(1.0));
    // circleVec.push_back(Circle(2.0));
    // circleVec.push_back(Circle(5.0));
    // double area = 0, perimeter = 0;
    // for (auto circle : circleVec)
    // {
    //     area += circle.getArea();
    //     perimeter += circle.getPerimeter();
    // }
    // std::cout << "We Have " << circleVec.size() << " Circles" << '\n'
    //           << "The Area is: " << area << '\n'
    //           << "The perimeter is: " << perimeter << std::endl;

    //First way to create an object and add it to vector could be achieved below
    Circle circle(2.0);
    circles.push_back(circle);
    //If we did not need the object, rather create the circle and add to vector in one step
    circles.push_back(Circle(1.0));

    //Finally, if we wanted to use an iterator to add the vector to a specific location http://www.cplusplus.com/reference/vector/vector/emplace/
    circles.emplace(circles.end(), Circle(5.0));

    double total_area = computeTotalArea(circles);
    std::cout << "The Total Area is " << total_area << std::endl;

    double total_perimeter = computeTotalPerimeter(circles);
    std::cout << "The Total Perimeter is " << total_perimeter << "\n"
              << std::endl;

    std::cout << "This is Q4 :Program that creates a vector of circles with random radii (the radii can be between 1.0 and 10.0) and uses the function to compute the combined area" << std::endl;
    // Setup random number generation using seed from system clock
    // We set up the seed and the distribution once, and thereafter simply draw from this distribution.
    int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    //https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
    std::uniform_real_distribution<double> dist(1.0, 10.0); //Produces random floating-point values i, uniformly distributed on the interval [a, b]  which for us is 1.0 to 10.0

    int num_circles;
    std::cout << "How many Circles would you like to generate : ";

    bool inRange = false;
    while (!inRange)
    {
        while (!(std::cin >> num_circles))
        {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input, Try again: ";
        }
        if (num_circles <= 0)
        {
            std::cout << "Value must be over " << 0 << ", Try again: ";
        }
        else
        {
            inRange = true;
        }
    }
    std::cout << std::endl;

    // Loop through and add a bunch of rectangles with random (1.0-10.0) sides
    for (int i = 0; i < num_circles; i++)
    {
        circles.push_back(Circle(dist(gen))); // Here we draw from the distribution, Each call to dis(gen) generates a new random double
    }

    // Compute the total area and print it
    double total_Area = computeTotalArea(circles);
    std::cout << "Area after: " << total_Area << std::endl;

    return 0;
}

double computeTotalArea(std::vector<Circle> circles)
{
    double total_Area = 0;
    for (auto circle : circles)
    {
        total_Area += circle.getArea();
    }
    return total_Area;
}
double computeTotalPerimeter(std::vector<Circle> circles)
{
    double total_perimeter = 0;
    for (auto circle : circles)
    {
        total_perimeter += circle.getPerimeter();
    }
    return total_perimeter;
}