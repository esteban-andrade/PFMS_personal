#include <iostream>
#include <deque>
#include <chrono>
#include <random>
#include <vector>
#include <bits/stdc++.h>
using namespace std;

//! Sample function for the creating elements of the deque
void populateDeque(std::deque<double> &values, int num_values, double mean, double std_dev)
{
    mean = 8;
    std_dev = 4;
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator(seed);
    normal_distribution<double> val(mean, std_dev);
    cout << "How many deques do you want to create" << endl;
    cin >> num_values;

    for (int i = 0; i < num_values; i++)
    {
        values.push_back(val(generator));
    }
}

//! Sample function for printing elements of the deque
void printDeque(std::deque<double> &values)
{
    // Loop through the deque and print the values
    for (auto value : values)
    {
        std::cout << value << " ";
    }
    std::cout << std::endl;
}
void populateVector(std::vector<double> &empty, int num_values, double mean, double std_dev)
{
    mean = 8;
    std_dev = 4;
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator(seed);
    normal_distribution<double> val(mean, std_dev);
    cout << "How many vectors do you want to create" << endl;
    cin >> num_values;
    for (int i = 0; i < num_values; i++)
    {
        empty.push_back(val(generator));
    }
}

//! Sample function for printing elements of the deque
void printVector(std::vector<double> &empty)
{
    // Loop through the deque and print the values
    for (auto value : empty)
    {
        std::cout << value << " ";
    }
    std::cout << std::endl;
}

void bubbleSort(vector<double> &empty)
{
    bool swapp = true;
    while (swapp)
    {
        swapp = false;
        for (int i = 0; i < empty.size() - 1; i++)
        {
            if (empty[i] > empty[i + 1])
            {
                empty[i] += empty[i + 1];
                empty[i + 1] = empty[i] - empty[i + 1];
                empty[i] -= empty[i + 1];
                swapp = true;
            }
        }
    }
}

int main()
{

    ////////////////////////////////////////////////

    std::deque<double> values;
    populateDeque(values, 0, 0, 0);
    printDeque(values);

    std::vector<double> empty;
    populateVector(empty, 0, 0, 0);
    printVector(empty);
    bubbleSort(empty);
    cout << "Arranged vector values: " << endl;
    printVector(empty);

    ////////////////////////////////////////////////

    // Bubble sort one of your containers

    // Print the contents of the container

    return 0;
}
