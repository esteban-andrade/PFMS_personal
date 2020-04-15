// We need to include the declaration of our new rectangle class in order to use it.
#include "camera.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <random>
#include <chrono>
#include <thread>

void userInputs(int &rows, int &columns, int &sample_time);
void verifyInputs(int &rows, int &columns, int &sample_time);
void displayData(std::vector<Camera> &vec, int &index);
int main()
{
    int set_rows;
    int set_columns;
    int set_sampling_time;
    int index = 0;

    // welcome user
    std::cout << "Welcome" << std::endl;
    std::cout << "Please enter user parameters" << std::endl;
    std::cout << "The program will determine the best configuration based on your inputs" << std::endl;
    std::cout << "Press Enter to Continue" << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    system("clear"); // clear console

    userInputs(set_rows, set_columns, set_sampling_time);   // call function to pass user parameters
    verifyInputs(set_rows, set_columns, set_sampling_time); // verify user parameters

    auto start = std::chrono::system_clock::now();
    // std::vector<Camera> output;
    // output.push_back(Camera(set_rows, set_columns, set_sampling_time));
    Camera data(set_rows,set_columns,set_sampling_time);
    data.timeTracker();
    while ((std::chrono::system_clock::now() - start) < std::chrono::seconds{5}) // while loop for time
    {
        index++;
        // displayData(output, index);
        // auto end = std::chrono::system_clock::now();
        // std::cout << "Time is " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " seconds\n" << std::endl;
        Camera data(set_rows,set_columns,set_sampling_time);
        data.pollData();
        std::vector<std::vector<int>> output = data.getData();
        data.setSampleNumber(index);
        data.getElapsedTime();
        std::cout << "Sample #" << data.getSampleNumber() << std::endl;
        // print sensing range
        for (int i = 0; i < output.size(); i++)
        {

            for (int j = 0; j < output.at(i).size(); j++)
            {

                std::cout << "[" << std::setw(3) << output.at(i).at(j) << "] ";
            }
            std::cout << std::endl;
        }
        std::cout << "Elapsed Time " << data.getElapsedTime() << " seconds" << std::endl;
        // std::cout << "\n"
        //           << std::endl;
    
    
    }
std::cout << "Time First Data Query " << data.getTimeFirstQuery().count() << " seconds" << std::endl;
        std::cout << "\n"
                  << std::endl;
    set_rows = 0, set_columns = 0, set_sampling_time = 0, index = 0;

    userInputs(set_rows, set_columns, set_sampling_time); // call function to pass user parameters
    verifyInputs(set_rows, set_columns, set_sampling_time);

    //output.clear(); // clear vector for new input.
    // output.push_back(Camera(set_rows, set_columns, set_sampling_time));
    while (true) // while loop for time
    {

        index++;
        // displayData(output, index);
        // auto end = std::chrono::system_clock::now();
        // std::cout << "Time since initial query " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " seconds \n" << std::endl;
        Camera data(set_rows,set_columns,set_sampling_time);
        data.pollData();
        std::vector<std::vector<int>> output = data.getData();
        data.setSampleNumber(index);
        data.getElapsedTime();
        std::cout << "Sample #" << data.getSampleNumber() << std::endl;
        // print sensing range
        for (int i = 0; i < output.size(); i++)
        {

            for (int j = 0; j < output.at(i).size(); j++)
            {

                std::cout << "[" << std::setw(3) << output.at(i).at(j) << "] ";
            }
            std::cout << std::endl;
        }
        std::cout << "Elapsed Time " << data.getTimeFirstQuery().count() << " seconds" << std::endl;
        // std::cout << "\n"
        //           << std::endl;
    }

    return 0;
}

void displayData(std::vector<Camera> &vec, int &index)
{
    for (auto data : vec)
    {
        data.pollData();
        std::vector<std::vector<int>> output = data.getData();
        data.setSampleNumber(index);
        data.getElapsedTime();
        std::cout << "Sample #" << data.getSampleNumber() << std::endl;
        // print sensing range
        for (int i = 0; i < output.size(); i++)
        {

            for (int j = 0; j < output.at(i).size(); j++)
            {

                std::cout << "[" << std::setw(3) << output.at(i).at(j) << "] ";
            }
            std::cout << std::endl;
        }
        std::cout << "Elapsed Time " << data.getElapsedTime() << " seconds" << std::endl;
        // std::cout << "\n"
        //           << std::endl;
    }
}

void userInputs(int &rows, int &columns, int &sample_time)
{
    std::cout << "Please enter the number of rows required for the image size: " << std::endl;

    while (!(std::cin >> rows) || rows < 0)
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input, Try again: ";
    }
    std::cout << "Please enter the number of columns required for the image size: " << std::endl;
    while (!(std::cin >> columns) || columns < 0)
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input, Try again: ";
    }
    std::cout << "Please enter the sampling required: " << std::endl;
    while (!(std::cin >> sample_time) || sample_time < 0)
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input, Try again: ";
    }
}

void verifyInputs(int &rows, int &columns, int &sample_time)
{
    Camera *test = new Camera(rows, columns, sample_time); // creation of testing object for user inputs
    while (test->queryParameters() == false)
    {
        system("clear"); // clear console
        std::cout << "Invalid Parameters. Please try again" << std::endl;
        std::cin.clear();
        std::cout << "Please enter the number of rows required for the image size: " << std::endl;

        while (!(std::cin >> rows) || rows < 0)
        {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input, Try again: ";
        }

        std::cout << "Please enter the number of columns required for the image size: " << std::endl;
        while (!(std::cin >> columns) || columns < 0)
        {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input, Try again: ";
        }

        std::cout << "Please enter the sampling required: " << std::endl;
        while (!(std::cin >> sample_time) || sample_time < 0)
        {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input, Try again: ";
        }
        // adjust new adjusted user parameters
        test->setImageSize(rows, columns);
        test->setSamplingTime(sample_time);
    }
    if (test->queryParameters() == true)
    {
        system("clear"); // clear console
        std::cout << "Based on Inputs the configuration is: " << std::endl;
        std::cout << "Image Size (Rows x Columns):" << test->getImageSize() << "\n"
                  << "Sampling time :" << test->getSamplingTime() << " miliseconds \n"
                  << std::endl;
        test->pollData();
        sample_time = test->getSamplingTime();
        columns = test->getColumnNumer();
        rows = test->getRowNumber();

        delete test;
    }
}