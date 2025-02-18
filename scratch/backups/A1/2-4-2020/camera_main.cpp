// We need to include the declaration of our new rectangle class in order to use it.
#include "camera.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <random>
#include <chrono>
#include <thread>

int main()
{
    int set_rows;
    int set_columns;
    int set_sampling_time;
    int index = 0;

    // welcome use
    std::cout << "Welcome" << std::endl;
    std::cout << "Please enter user parameters" << std::endl;
    std::cout << "The program will determine the best configuration based on your inputs" << std::endl;
    std::cout << "Press Enter to Continue" << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    system("clear"); // clear console

    std::cout << "Please enter the number of rows required for the image size: " << std::endl;

    while (!(std::cin >> set_rows) || set_rows < 0)
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input, Try again: ";
    }
    std::cout << "Please enter the number of columns required for the image size: " << std::endl;
    while (!(std::cin >> set_columns) || set_columns < 0)
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input, Try again: ";
    }
    std::cout << "Please enter the sampling required: " << std::endl;
    while (!(std::cin >> set_sampling_time) || set_sampling_time < 0)
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input, Try again: ";
    }

    Camera *test = new Camera(set_rows, set_columns, set_sampling_time); // creation of testing object for user inputs

    while (test->queryParameters() == false)
    {
        system("clear"); // clear console
        std::cout << "Invalid Parameters. Please try again" << std::endl;
        std::cin.clear();
        std::cout << "Please enter the number of rows required for the image size: " << std::endl;

        while (!(std::cin >> set_rows) || set_rows < 0)
        {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input, Try again: ";
        }

        std::cout << "Please enter the number of columns required for the image size: " << std::endl;
        while (!(std::cin >> set_columns) || set_columns < 0)
        {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input, Try again: ";
        }

        std::cout << "Please enter the sampling required: " << std::endl;
        while (!(std::cin >> set_sampling_time) || set_sampling_time < 0)
        {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input, Try again: ";
        }
        // adjust new adjusted user parameters
        test->setImageSize(set_rows, set_columns);
        test->setSamplingTime(set_sampling_time);
    }
    if (test->queryParameters() == true)
    {
        system("clear"); // clear console
        std::cout << "Based on Inputs the configuration is: " << std::endl;
        test->getData();
        std::cout << "Image Size (Rows x Columns):" << test->getImageSize() << "\n"
                  << "Sampling time :" << test->getSamplingTime() << " miliseconds \n"
                  << std::endl;
        set_sampling_time = test->getSamplingTime();
        set_columns = test->getColumnNumer();
        set_rows = test->getRowNumber();

        delete test;

    }
        auto start = std::chrono::system_clock::now();
        Camera data(set_rows, set_columns, set_sampling_time);                       // creation valid object of object
        
        while ((std::chrono::system_clock::now() - start) < std::chrono::seconds{5}) // while loop for time
        {

            Camera data(set_rows, set_columns, set_sampling_time); // creation valid object of object
            index++;
            data.setSampleNumber(index);
            data.getData();
            std::vector<std::vector<int>> output = data.printData();

            std::cout << "Sample #" << data.getSampleNumber() << std::endl;
            // print sensing range
            for (int i = 0; i < output.size(); i++)
            {
                for (int j = 0; j < output.at(i).size(); j++)
                {
                    std::cout << "[" << output.at(i).at(j) << "] ";
                }
                std::cout << std::endl;
            }
            std::cout << "The Sampling time is: " << data.getSamplingTime() << " miliseconds \n"
                      << std::endl;
        }
        auto end = std::chrono::system_clock::now();

        std::cout << "Elapsed time is " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " seconds" << std::endl;
    
        
    return 0;
}

// test.getData();
//         std::vector<std::vector<int>> output = test.printData();

//         std::cout << "Sample #" << test.getSampleNumber() << std::endl;
//         // print sensing range
//         for (int i = 0; i < output.size(); i++)
//         {
//             for (int j = 0; j < output.at(i).size(); j++)
//             {
//                 std::cout << "[" << output.at(i).at(j) << "] ";
//             }
//             std::cout << std::endl;
//         }
//         std::cout << "The Sampling time is: " << test.getSamplingTime() << " miliseconds \n"
//                   << std::endl;

/*
for (int i=5; i>0; --i) {
    std::this_thread::sleep_for (std::chrono::seconds(1));
  }
    

*/

// while (test.queryParameters() == false)
// {
//      system("clear"); // clear console
//     std::cout << "Invalid Parameters. Please try again" << std::endl;
//     std::cin.clear();
//     std::cout << "Please enter the number of rows required for the image size: " << std::endl;

//     while (!(std::cin >> set_rows) || set_rows < 0)
//     {
//         std::cin.clear();
//         std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
//         std::cout << "Invalid input, Try again: ";
//     }

//     std::cout << "Please enter the number of columns required for the image size: " << std::endl;
//     while (!(std::cin >> set_columns) || set_columns < 0)
//     {
//         std::cin.clear();
//         std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
//         std::cout << "Invalid input, Try again: ";
//     }

//     std::cout << "Please enter the sampling required: " << std::endl;
//     while (!(std::cin >> set_sampling_time) || set_sampling_time < 0)
//     {
//         std::cin.clear();
//         std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
//         std::cout << "Invalid input, Try again: ";
//     }
//     // adjust new adjusted user parameters
//     test.setImageSize(set_rows, set_columns);
//     test.setSamplingTime(set_sampling_time);
// test .clear()
// }
// if (test.queryParameters() == true)
// {
//      system("clear"); // clear console
//     std::cout << "Based on Inputs the configuration is: " << std::endl;
//     test.getData();
//     std::cout << "Image Size (Rows x Columns):" << test.getImageSize() << "\n"
//               << "Sampling time :" << test.getSamplingTime() << " miliseconds" << std::endl;

// }
