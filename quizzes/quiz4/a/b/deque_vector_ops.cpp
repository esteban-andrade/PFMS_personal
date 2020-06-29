#include <iostream>
#include <deque>
#include <chrono>
#include <random>
#include <vector>

//! Sample function for the creating elements of the deque
void populateDeque(std::deque<double> &values, int num_values, double mean, double std_dev)
{

	// Create a random number generator and seed it from
	// the system clock so numbers are different each time

	//http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	for (int i = 0; i < num_values; i++)
	{
		std::normal_distribution<double> distribution(mean, std_dev);
		values.push_back(distribution(generator));
	}
}
void populateVector(std::vector<double> &vector_values, int num_values, double mean, double std_dev)
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	for (int i = 0; i < num_values; i++)
	{
		std::normal_distribution<double> distribution(mean, std_dev);
		vector_values.push_back(distribution(generator));
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

void printVector(std::vector<double> &vector_values)
{
	for (auto value : vector_values)
	{
		std::cout << value << " ";
	}
	std::cout << std::endl;
}
void bubbleSortDeque(std::deque<double> &values)
{
	double place_holder = 0;
	for (int i = 0; i < values.size(); i++)
	{
		for (int j = i + 1; j < values.size(); j++)
		{
			if (values.at(j) < values.at(i))
			{
				place_holder = values.at(i);
				values.at(i) = values.at(j);
				values.at(j) = place_holder;
			}
		}
	}
}
void bubbleSortVector(std::vector<double> &vector_values)
{
	bool swapped;
	double place_holder;
	for (int i = 0; i < vector_values.size() - 1; i++)
	{
		swapped = false;
		for (int j = 0; j < vector_values.size() - i - 1; j++)
		{
			if (vector_values.at(j) > vector_values.at(j + 1))
			{
				place_holder = vector_values.at(j);
				vector_values.at(j) = vector_values.at(j + 1);
				vector_values.at(j + 1) = place_holder;
				swapped = true;
			}
		}
		if (swapped == false)
		{
			break;
		}
	}
}
int main()
{

	////////////////////////////////////////////////

	// Create an empty deque
	std::deque<double> values;
	// Populate it with random numbers
	static const double mean = 8;
	static const double std_dev = 4;
	int user_input;
	std::cout << "Please enter number of elements that are required to be generated" << std::endl;
	while (!(std::cin >> user_input) || user_input <= 0) // check that input is valid
	{
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		std::cout << "Invalid input, Try again: ";
	}
	populateDeque(values, user_input, mean, std_dev);
	// Print the contents of the deque
	std::cout << "Unsorted Values Generated DEQUE" << std::endl;
	printDeque(values);
	////////////////////////////////////////////////

	// Create an empty vector
	std::vector<double> vector_values;
	// Populate it with random numbers
	populateVector(vector_values, user_input, mean, std_dev);
	// Print the contents of the vector
	std::cout << "Unsorted Values Generated VECTOR" << std::endl;
	printVector(vector_values);
	////////////////////////////////////////////////

	// Bubble sort one of your containers
	bubbleSortDeque(values);
	// Print the contents of the container
	std::cout << "\nSorted Values DEQUE" << std::endl;
	printDeque(values);

	bubbleSortVector(vector_values);
	std::cout << "\nSorted Values VECTOR" << std::endl;
	printVector(vector_values);
	return 0;
}
