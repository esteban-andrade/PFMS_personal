#include <iostream>
#include <deque>
#include <chrono>
#include <random>
#include <vector>

//! Sample function for the creating elements of the deque
void populateDeque(std::deque<double>& values, int num_values, double mean, double std_dev) {

	// Create a random number generator and seed it from
	// the system clock so numbers are different each time

    //http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/

}

//! Sample function for printing elements of the deque
void printDeque(std::deque<double>& values) {
	// Loop through the deque and print the values
	for (auto value : values) {
		std::cout << value << " ";
	}
	std::cout << std::endl;
}

int main() {

    ////////////////////////////////////////////////

	// Create an empty deque
	std::deque<double> values;
	// Populate it with random numbers

	// Print the contents of the deque

    ////////////////////////////////////////////////

	// Create an empty vector

	// Populate it with random numbers

	// Print the contents of the vector

    ////////////////////////////////////////////////


    // Bubble sort one of your containers

	// Print the contents of the container


	return 0;
} 
