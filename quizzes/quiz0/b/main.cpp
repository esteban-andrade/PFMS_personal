// We need to include the declaration of our new rectangle class in order to use it.
#include "sample.h"
#include <iostream>

int main()
{
    double value = 2.5;
    std::cout << "The passed value is " << value << std::endl;

    // create a sample object and passes a value
    Sample sample(value);
    std::cout << "The returned value is " << sample.readvalue() << "\n"
              << "This was performed by passing a value with the constructor"
              << "\n"
              << std::endl;

    value = 7.9;
    std::cout << "The value is changed to : " << value << std::endl;

    // This is used to assign a different value using the method
    sample.setvalue(value);
    std::cout << "The returned value is " << sample.readvalue() << "\n"
              << "This was perfomed by passing a different value with the method" << std::endl;

    return 0;
}
