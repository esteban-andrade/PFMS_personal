#include <iostream>
#include <cmath>
#include <vector>  // include vector STL
#include <sstream> // to get the size of if passes via command line

//Q5  create function that accepts a double as a paremeter and returns a bool value is if the double is greater than zero and the square value instead of the original

bool q5(double &num)
{
    int square;
    bool condition = false;
    if (num > 0)
    {
        condition = true;
    }
    square = std::pow(num, 2);
    return condition;
}

//Q6 Create function that accepts a double as a paremeter and returns a bool value is if the double is greater than zero and the square value, the cube value and the passed value incremeted by one
bool q6(double &num, double &square, double &cube)
{

    bool condition = false;
    if (num > 0)
    {
        condition = true;
    }
    square = std::pow(num, 2);
    cube = std::pow(num, 3);
    num++;
    return condition;
}

/* Q7

    Create a structure called Sensor than contains
    A variable for the number of samples num_samples
    An array of samples double data[]
    Create a sensor from the struct
    Populate data with 5 elements, each data[i] =i. How to code end of loop?
    Create a function that prints samples
    Can you initialise a sensor of two elements in one line of code?

*/

static const int max_size = 10;
struct Sensor
{
    int num_samples;
    double data[max_size];
};

void print_struct(Sensor sensor)
{
    for (unsigned int i = 0; i < sensor.num_samples; i++)
    {
        std::cout << sensor.data[i] << " ";
    }
    std ::cout << std::endl;
}

/* Q8

    Create a vector of integers called data of length 10 elements
    Create a loop to populate elements of data (each element [i] =i))
    Loop to print elements of vector Questions:
    How to create a vector of floats?
    How could we pass the length to the executable instead of hard coding 10?

*/
void q8(/*int argc, char **argv */)
{
    int lenght = 10;
    // if (argc > 1)
    // {
    //     //! Obtain value of vector size if supplied via command line
    //     //! ie running the exectubale (./vectors 5) will create a vector with 5 elements (otherwise default is 10)
    //     std::istringstream ss(argv[1]);
    //     if (!(ss >> lenght))
    //     {
    //         std::cerr << "Invalid Number :" << argv[1] << "length of zero used\n";
    //     }
    //     else if (!ss.eof())
    //     {
    //         std::cerr << "Trailing characters aftr number :" << argv[1] << " using " << lenght << "\n";
    //     }
    // }

    // vector

    std::vector<int> data; // create vector of integers called data

    for (int i = 0; i < lenght; i++)
    { // create a loop to populate the elements of data. Each element [i] =i
        data.push_back(i);
    }

    for (int i = 0; i < data.size(); i++)
    { // loop to print elements of vector. Access each element in location
        std::cout << data.at(i) << " ";
    }
    std::cout << std::endl;

    for (auto elem : data)
    { // loop tp print elements of vector with auto
        std::cout << elem << " ";
    }
    std::cout << std::endl;
}

/* Q9

    Create a vector of vectors containing integers called data
    Create a loop to populate elements of data such that is is a matrix of 4x4 elements (each element row =i))
    Loop to print elements of data Questions:
    What else could you store in this container?

*/

// void q9()
// {
//     {
//         std::vector<std::vector<int>> data; // vectors of vectors containing integers
//                                             //create a loop to populate  elements of data such that is a matrix 4x4 elements
//         const int rows = 4;
//         const int cols = 4;
//         for (int i = 0; i < rows; i++)
//         {
//             std::vector<int> row;
//             for (int j = 0; j < cols; i++)
//             {
//                 row.push_back(j);
//             }
//             data.push_back(row);
//         }
//         std::cout << "MATRIX" << std::endl;

//         // loop to print element of data

//         for (int i = 0; i < data.size(); i++)
//         {
//             for (int j = 0; j < data.at(i).size(); j++)
//             {
//                 std::cout << data.at(i).at(j) << " ";
//             }
//             std::cout << std::endl;
//         }
//     }
// }

void q9()
{
    {
        //* Create a vector of vectors containing integers called  `data`
        std::vector<std::vector<int>> data;

        //* Create a loop to populate elements of `data` such that is is a matrix of 4x4 elements (each element row =i))
        const int rows = 4; //We have 4 rows
        const int cols = 4; //We have 4 cols

        for (int i = 0; i < rows; i++)
        {
            std::vector<int> row;
            for (int j = 0; j < cols; j++)
            {
                row.push_back(j);
            }
            data.push_back(row);
        }

        std::cout << "MATRIX" << std::endl;

        //* Loop to print elements of `data`

        for (int i = 0; i < data.size(); i++)
        {
            for (int j = 0; j < data.at(i).size(); j++)
            {
                std::cout << data.at(i).at(j) << " ";
            }
            std::cout << std::endl;
        }
    }
}

int main()
{

    double a = 9.0;
    bool result;
    std::cout << "This is question 5"
              << std::endl;
    std::cout << "The initial value " << a << std::endl;
    result = q5(a);
    std::cout << "Is the value Positive"
              << " " << result << std::endl;
    std::cout << "The result value is " << a << std::endl;
    std::cout << "This is  end of question 5\n"
              << std::endl;

    std::cout << "This is question 6"
              << std::endl;

    double b = 4;
    double square, cube;
    std ::cout << "the given value is" << b << std::endl;
    result = q6(b, square, cube);
    std ::cout << "is the value positive" << result << std::endl;
    std ::cout << "The given value +1 is :" << b << std::endl;
    std ::cout << "The square value is " << square << std::endl;
    std ::cout << "the cube value is " << cube << std::endl;
    std::cout << "This is  end of question 6\n"
              << std::endl;

    std::cout << "This is question 7"
              << std::endl;

    Sensor sensor;
    sensor.num_samples = 5;
    for (int i = 0; i < sensor.num_samples; i++)
    {
        sensor.data[i] = i;
        std::cout << " { " << sensor.data[i] << " } " << std::endl;
    }
    print_struct(sensor); // funtion called to print struct

    // this is known as list initalization (aggregate initialisation)
    Sensor sensor2{2, {7, -12}}; // create data of size two with elements noted
    print_struct(sensor2);
    std::cout << "This is  end of question 7\n"
              << std::endl;

    std::cout << "This is question 8"
              << std::endl;
    q8();
    std::cout << "This is  end of question 8\n"
              << std::endl;

    std::cout << "This is question 9"
              << std::endl;
    q9();
    

    std::cout << "This is  end of question 9\n"
              << std::endl;

    return 0;
}