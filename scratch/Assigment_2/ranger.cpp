#include "ranger.h"
#include <iostream>
#include <random>
#include <chrono>

//sample data for each sensor
static const double mean = 4.0;
static const double std_deviation = 5.0;

//ranger default constructor. Will set paramaters as to a non-identified sensor
Ranger::Ranger() : offset_(0), model_("Sensor Not Specified"), min_range_(0), max_range_(0), field_of_view_(0), angular_resolution_(0), number_samples_(0) {}

std::vector<double> Ranger::generateData() // generates vector with data
{
    data_.clear();                      //clears the vector. Ensures its empty
    data_.reserve(number_samples_ + 1); //reserves adequate amount of memeory based on the number of samples

    // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::random_device seed;
    std::default_random_engine generator(seed());
    std::normal_distribution<double> distribution(mean, std_deviation); //genarates data based on the normal distribution
    for (int i = 0; i < number_samples_; i++)
    {
        double reading = distribution(generator);

        if (reading < min_range_) //if data is on below the minimum range it will cap the distance to minimum. Data could be on the edge thus has to be accepted
        {
            reading = min_range_;
        }
        else if (reading > max_range_) //if data is on above  the maximum range it will cap the distance to maximum. Data could be on the edge thus has to be accepted
        {
            reading = max_range_;
        }
        data_.push_back(reading); // store data im container
    }
    return data_; // return generated data
}
std::vector<double> Ranger::getData()
{
    return data_; // return data that was already generated
}
unsigned int Ranger::getAngularResolution(void)
{
    return angular_resolution_; // returns angular resolution
}

int Ranger::getOffset(void)
{
    return offset_; // returns offset
}
unsigned int Ranger::getFieldOfView()
{
    return field_of_view_; //returns field if view
}

double Ranger::getMaxRange(void)
{
    return max_range_; //return max range
}
double Ranger::getMinRange(void)
{
    return min_range_; //returns min  range
}

SensingMethod Ranger::getSensingMethod(void)
{
    return sensing_type_; // returns sensing type
}

std::string Ranger::getModel()
{
    return model_; //returns sensor model
}
unsigned int Ranger::getNumberSamples(void)
{
    return number_samples_; //returns number of samples
}

bool Ranger::setAngularResolution(unsigned int angular_resolution)
{
    if (angular_resolution >= 0) //check if angular resolution is  valid
    {
        angular_resolution_ = angular_resolution; // set angular resolution
        return true;
    }
    else
    {
        return false;
    }
}
bool Ranger::setOffset(int offset)
{
    if (offset >= -360 && offset <= 360) //check that offset canbe 360 or -360. Thus an entire revolution in clockwise or counter-clockwise
    {
        offset_ = offset; //set offset
        return true;
    }
    else
    {
        return false;
    }
}
bool Ranger::setFieldOfView(unsigned int field_of_view)
{
    if (field_of_view <= 180 || field_of_view >= -180) // checks that field of view can be between 0 and 360. 180 degrees clockwise and counter-clockwise
    {
        field_of_view_ = field_of_view; // sets field of view
        return true;
    }
    else
    {
        return false;
    }
}

bool Ranger ::setNumberOfSamples(unsigned int samples)
{
    if (samples >= 0) // checks that a valid number of samples is used
    {
        number_samples_ = samples;
        return true;
    }
    else
    {
        return false;
    }
}
bool Ranger::setModel(std::string model)
{
    if (model != " ") // checks that model strings is not empty
    {
        model_ = model;
        return true;
    }
    else
    {
        false;
    }
}
bool Ranger ::setMaxDistance(double distance)
{
    if (distance > 0) //checks that the distance is valid
    {
        max_range_ = distance; // sets max range
        return true;
    }
    else
    {
        return false;
    }
}
bool Ranger ::setMinDistance(double distance)
{
    if (distance > 0) //checks that distance is valid
    {
        min_range_ = distance; // set min range
        return true;
    }
    else
    {
        return false;
    }
}
SensingMethod Ranger::setSensingMethod(SensingMethod type)
{
    if (type == CONE || type == POINT) // checks that sensing type is valid
        sensing_type_ = type; // sets the sensor sensing type
}