// We need to include the declaration of our new rectangle class in order to use it.
#include "sample.h"
#include <iostream>

int main () {

    // Create a rectangle object
    //

    // Set the values of the sides
    //rectangle.setWidthHeight(5.0,5.0);

    // Get the area and print it to screen
    //double result = rectangle.area();
    //std::cout << result << std::endl;

    Sample sample = 10;
    std::cout << "sample = " << sample.readvalue() << std::endl;

    return 0;
}
