#include <iostream> // Includes std::cout and friends so we can output to console
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <limits>
#include <vector>
#include <cmath>

// Create a macro (#define) to represent the max possible array size
#define ARRAY_MAX_SIZE 100

// function to print elements of the array                                //  both functions have the same definition
void printArray(double x[], int xSize)
{
    for (int i = 0; i < xSize; i++)
    {
        std::cout << "x[" << i << "] = " << x[i] << std::endl;
    }
}

// function to populate array with random numbers
void populateWithRandomNumbers(double num_array[], int &array_size, int num_elements)
{

    //we select a seed for the random generator, so it is truly random (neve the same seed)
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a uniform distribution between 0 and 10 and draw elements from it
    std::uniform_real_distribution<> value_distribution(0, 10.0);
    // generate the required amount of random numbers
    for (int i = 0; i < num_elements; i++)
    {
        num_array[i] = value_distribution(generator);
    }
    //let's update the array size
    //array_size += num_elements;
}

//prints only the elements of the array that are larger than : mean + one standard deviation
void meanPlus_stdDeviation(double array[], int n)
{
    double mean, sum, variance, stdDeviation;
    for (int i = 0; i < n; i++)
    {
        sum += array[i];
    }
    mean = sum / n;
    for (int i = 0; i < n; i++)
    {
        variance += pow(array[i] - mean, 2);
    }
    variance = variance / n;
    stdDeviation = sqrt(variance);

    std::cout << "The mean is : " << mean << " " << std::endl;
    std::cout << "The standard deviation is : " << stdDeviation << " " << std::endl;
    std::cout << "The standard deviation + mean : " << stdDeviation + mean << " " << std::endl;

    std::cout << "The elements that larger than mean + one standard deviation are: \n";
    for (int i = 0; i < n; i++)
    {
        if (array[i] > mean + stdDeviation)
            std::cout << " {" << array[i] << "} ";
    }
    std::cout << std::endl;
}

// function that will convert the array to vector
void vectorConversion(double array[], int n)
{
    std::vector<double> vec(array, array + n);
    std::cout << "The elements of the array converted to vector will be : \n";
    for (auto i : vec)
    {
        std::cout << " {" << i << "} ";
    }
    std::cout << std::endl;
}

// Every executable needs a main function which returns an int
int main()
{

    // Create an array x of doubles with 10 elements
    int arraySize = 10;
    // Populate the elements of array on creating of array, each element [i] has value i (INITIALISER LIST)
    double x[ARRAY_MAX_SIZE] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

    // Print array
    printArray(x, arraySize);

    // Ask user to specify how many additional numbers are to be generated
    int num;
    std::cout << "How many random elements do you wish to generate : ";
    while (!(std::cin >> num) || num < 0 || num > 100)
    {
        if (!(std::cin))
        {
            std::cout << "Invalid input, Try again: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            
        }
        else if (num < 0)
         std::cout << "Invalid input, Please enter a valid positive number : ";

        else if (num > 100)
          std::cout << "Exceeded size, Please enter value less or equal to than 100 : ";
    }

    // create a  new array with new allocation of memory. This will fix the segmentation error. Another method would be to use vectors instead of arrays.
    // new array that will hold the new generated number
    int generatedArraySize = num;
    double *new_x = new double[generatedArraySize];

    // Populate new array with random numbers
    populateWithRandomNumbers(new_x, generatedArraySize, num);

    //print new generated array
    std::cout << " The elements from the new elements array are :" << std::endl;
    printArray(new_x, generatedArraySize);

    // create a new concatonated array that will contain all the new and previous array
    double *finalArray = new double[arraySize + generatedArraySize];
    std::copy(x, x + arraySize, finalArray);
    std::copy(new_x, new_x + generatedArraySize, finalArray + arraySize);

    // Print final array
    std::cout << " The elements from the concatonated array are :" << std::endl;
    printArray(finalArray, generatedArraySize + arraySize);

    // print mean+ stdDeviation
    meanPlus_stdDeviation(finalArray, generatedArraySize + arraySize);

    // print transformation to vectors
    vectorConversion(finalArray, generatedArraySize + arraySize);

    // Main function should return an integer
    return 0;
}
