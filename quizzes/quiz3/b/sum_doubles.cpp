#include <iostream>
#include <vector>
#include <random>
#include <iterator>

using std::cout;
using std::endl;
using std::vector;

// Declare the sum function ahead of main, normally we would do this in a header
// Use of & (ampersand) means we pass `numbers` by reference, avoiding a copy
// The risk of using a reference is we can change the original object
// We eliminate this risk by using the const keyword
double sum(const vector<double> &numbers);

int main()
{
    //TODO Create a vector of doubles with 4 values
    std::random_device generator;
    std::uniform_real_distribution<double> distribution(0, 100);
    vector<double> test;
    test.push_back(distribution(generator));
    test.push_back(distribution(generator));
    test.push_back(distribution(generator));
    test.push_back(distribution(generator));

    //TODO Add a value to the end/back
    test.insert(test.begin(), distribution(generator)); // add value to front of vector

    //TODO Modify the 3rd value
    test[2] = distribution(generator); // keep in mind it starts the count from 0 thus element 3 is vect[2]

    //TODO Print out the numbers
    // Using a Range-based for loop with the auto keyword
    for (auto vec : test)
    {
        cout << "values \t"
             << vec << endl;
    }

    //TODO Compute the sum via sun function and print sum
    double result = sum(test);
    cout << "Total sum " << result << endl;

    return 0;
}

// Define the sum function, the signature must match exactly
double sum(const vector<double> &numbers)
{
    double total = 0.0;
    //TODO Use an iterator
    //total = std::accumulate(numbers.begin(), numbers.end(), 0);
    for (auto it = std::begin(numbers); it != std::end(numbers); it++)
    {
        //std::cout << " Iterator values " << *it << " " << std::endl;
    }
    total = std::accumulate(std::begin(numbers), std::end(numbers), 0.0);

    return total;
}
