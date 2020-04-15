#include "camera.h"

#include <cmath>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <thread>



const int min_pixel_value = 0;   // Min value =0
const int max_pixel_value = 255; // Max value 2^8 -1 = 256-1 = 255
const int min_row_size = 5;
const int min_column_size = 4;
const int max_row_size = 25;
const int max_column_size = 16;
const int min_sampling_time = 100;
const int max_sampling_time = 2000;

Camera::Camera() : rows_(5), columns_(4), sampling_time_(100)
{
    model_ = "CCD-MN0";
}

Camera::Camera(int &rows, int &columns, int &sampling_time)
{
    rows_ = rows;
    columns_ = columns;
    sampling_time_ = sampling_time;
    model_ = "CCD-MN0";
    start_config_time = std::chrono::system_clock::now();
    
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
    
    for (int i = 0; i < image_size_.size() - 1; i++)
    {
        for (int j = 0; j < image_size_.at(i).size(); j++)
        {
            image_size_.at(i + 1).at(j) = image_size_.at(i).at(j) / 2;
            
        }
    }
    auto end_time = std::chrono::system_clock::now();
    elapsed_time_= std::chrono::duration_cast<std::chrono::seconds>(end_time - start_config_time).count();
   
}
void Camera::timeTracker(void){
    //timer = std::chrono::duration_cast<std::chrono::duration<double>>();
}
void Camera::setSampleNumber(int sample_number)
{
    sample_number_ = sample_number;
}

bool Camera::queryParameters(void)
{
    if (rows_ == 5 || columns_ == 4 || sampling_time_ == 100)
    {
        rows_ = 5;
        columns_ = 4;
        sampling_time_ = 100;
        return true;
    }
    else if (rows_ == 25 || columns_ == 16 || sampling_time_ == 2000)
    {
        rows_ = 25;
        columns_ = 26;
        sampling_time_ = 2000;
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
    if (rows_ == 5 || columns_ == 4 || sampling_time_ == 100)
    {

        return "5x4";
    }
    else if (rows_ == 25 || columns_ == 16 || sampling_time_ == 2000)
    {

        return "25x16";
    }
}

int Camera::getElapsedTime()
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

// const std::vector<std::vector<int>> &Camera ::GetVect() const
// {
//     return image_size_;
// }

