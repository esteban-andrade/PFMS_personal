/*
    Created by Esteban Andrade 12824583
    Sensor 3: Monochrome Camera
*/


#include "camera.h"

#include <cmath>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <thread>

//Declaration of constants.

const int min_pixel_value = 0;   // Min value =0
const int max_pixel_value = 255; // Max value 2^8 -1 = 256-1 = 255
const int min_row_size = 5;
const int min_column_size = 4;
const int max_row_size = 25;
const int max_column_size = 16;
const int min_sampling_time = 100;
const int max_sampling_time = 2000;

Camera::Camera() : rows_(min_row_size), columns_(min_column_size), sampling_time_(min_sampling_time)
{
    model_ = "CCD-MN0";
}

Camera::Camera(int &rows, int &columns, int &sampling_time)
{
    rows_ = rows;
    columns_ = columns;
    sampling_time_ = sampling_time;
    model_ = "CCD-MN0";
    // track time since configs are change or set.
    start_config_time_ = std::chrono::system_clock::now();          
}
Camera::~Camera() {}

void Camera::pollData(void)
{

    // set Randomizer based on pixel
    int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<> dist(min_pixel_value, max_pixel_value);

    std::this_thread::sleep_for(std::chrono::milliseconds(sampling_time_));
    // generating output image
    for (int i = 0; i < rows_; i++)
    {
        std::vector<int> row;
        for (int j = 0; j < columns_; j++)
        {
            row.push_back(dist(generator));
        }
        image_size_.push_back(row);
    }
    // Adjust generated Data to match specifications 
    for (int i = 0; i < image_size_.size() - 1; i++)
    {
        for (int j = 0; j < image_size_.at(i).size(); j++)
        {
            image_size_.at(i + 1).at(j) = image_size_.at(i).at(j) / 2;
        }
    }
    auto end_time = std::chrono::system_clock::now();
    //  track the time since the configuration was set
    elapsed_time_ = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_config_time_).count();     
}
std::chrono::duration<double> Camera::getTimeFirstQuery()
{   
    // track time since Camera starting polling data.
    std::chrono::duration<double> time_spam = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - timer_tracker_);
    return time_spam;
}
void Camera::timeTracking(void)
{   
    // keep track time since sensor was initialized
    timer_tracker_ = std::chrono::steady_clock::now();
}

void Camera::setSampleNumber(int sample_number)
{
    sample_number_ = sample_number;
}

bool Camera::queryParameters(void)
{   
    // return a bool to check if parameters are acceptable or they need to be changed again
    if (rows_ == min_row_size || columns_ == min_column_size || sampling_time_ == min_sampling_time)
    {
        rows_ = min_row_size;
        columns_ = min_column_size;
        sampling_time_ = min_sampling_time;
        return true;
    }
    else if (rows_ == max_row_size || columns_ == max_column_size || sampling_time_ == max_sampling_time)
    {
        rows_ = max_row_size;
        columns_ = max_column_size;
        sampling_time_ = max_sampling_time;
        return true;
    }
    else
    {
        return false;
    }
}

int Camera ::getSampleNumber(void)
{
    return sample_number_;
}
int Camera ::getRowNumber(void)
{
    return rows_;
}
int Camera::getColumnNumer(void)
{
    return columns_;
}

std::vector<std::vector<int>> Camera::getData()
{
    return image_size_;
}

std::string Camera::getModel(void)
{
    return model_;
}
std::string Camera ::getImageSize(void)
{
    if (rows_ == min_row_size || columns_ == min_column_size || sampling_time_ == min_sampling_time)
    {

        return "5x4";
    }
    else if (rows_ == max_row_size || columns_ == max_column_size || sampling_time_ == max_sampling_time)
    {

        return "25x16";
    }
}

double Camera::getElapsedTime()
{
    return elapsed_time_;
}

int Camera ::getSamplingTime(void)
{
    return sampling_time_;
}
void Camera::setImageSize(int rows, int columns)
{
    rows_ = rows;
    rows_ = columns;
}
void Camera::setSamplingTime(int sampling_time)
{
    sampling_time_ = sampling_time;
}


