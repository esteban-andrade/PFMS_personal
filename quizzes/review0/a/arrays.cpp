#include <iostream> // Includes std::cout and friends so we can output to console
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <limits>
#include <cmath>
#include <vector>
using namespace std;

// Create a macro (#define) to represent the max possible array size
#define ARRAY_MAX_SIZE 100

# define my_sizeof(type) ((char *)(&type+1)-(char*)(&type)) 

// function to print elements of the array
/*void printArray(double* x, int xSize){
  for (double *ip = x; ip<(x+xSize); *ip++){
      std::cout << "*ip = " << *ip << std::endl;
  }
}*/

// function to print elements of the array
void printArray(double x[], int xSize){
  for (int i = 0; i<xSize; i++) {
      std::cout << "x[" << i << "] = " << x[i] << std::endl;

  }
}

// function to populate array with random numbers
void populateWithRandomNumbers(double num_array[], int& array_size, int num_elements) {

    //we select a seed for the random generator, so it is truly random (neve the same seed)
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a uniform distribution between 0 and 10 and draw elements from it
    std::uniform_real_distribution<> value_distribution(0,10.0);
    // generate the required amount of random numbers
    for (int i=array_size; i<array_size+num_elements; i++) {
        num_array[i] = value_distribution(generator);
    }
    //let's update the array size
    array_size+=num_elements;
}

void PrintFunction(double num_array[], int array_size){
    double mean = 0.0;
    double standarddevbrackets = 0.0;
    double standarddev = 0.0;

    for(int i = 0; i < array_size; i++){
        mean += (double) num_array[i];
    }
    mean = mean/array_size;
    for(int i = 0; i < array_size; i++){
         standarddevbrackets += ((double)num_array[i] - (double)mean)*((double)num_array[i] - (double)mean);
    }
    standarddevbrackets = standarddevbrackets*(1/((double) array_size));
    standarddev = (double)sqrt(standarddevbrackets);
    std::cout << "standard deviation = " << standarddev << ", mean = " << mean << "\nArray elements greater than standard deviation + mean:" << std::endl;
    for(int i = 0; i <= array_size; i++){
        if(num_array[i] >= (mean + standarddev))std::cout << i << ", " <<std::endl;
    }

}

void ConvertToVector(double num_array[], int arraySize, vector<double> & vs){
    for(int i = 0; i < arraySize; i++){
        vs.push_back(num_array[i]);
    }
}

// Every executable needs a main function which returns an int
int main () {

    // Create an array x of doubles with 10 elements
    int arraySize=10;
    // Populate the elements of array on creating of array, each element [i] has value i (INITIALISER LIST)
    double x[ARRAY_MAX_SIZE] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

    // Print array
    printArray(x,arraySize);

    // Ask user to specify how many additional numbers are to be generated
    int num;
    std::cout<<"How many random elements do you wish to generate : ";
    while(!(std::cin >> num) || (num > (ARRAY_MAX_SIZE-arraySize)) || num < 0){
         std::cin.clear();
         std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
         std::cout << "Invalid input, Try again: ";
    }
    // Populate array with random numbers
    populateWithRandomNumbers(x,arraySize,num);

    // Print array
    printArray(x,arraySize);
    PrintFunction(x,arraySize);

    //Declare vector
    vector<double> vs;

    //Convert array to vector
    ConvertToVector(x,arraySize,vs);

    //Print vector members
    for(double &v : vs){
        std::cout << "vs = " << v << std::endl;
    }
    // Main function should return an integer
    return 0;
}



