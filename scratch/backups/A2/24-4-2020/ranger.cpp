#include "ranger.h"
#include <iostream>
#include <random>
#include <chrono>

static const double mean = 4.0;
static const double std_deviation = 5.0;

Ranger::Ranger() : offset_(0), model_("Sensor Not Specified") {}

std::vector<double> Ranger::generateData()
{
    data_.clear();
    data_.reserve(number_samples_ + 1);

    // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    //std::default_random_engine generator(raw_data_seed_);
    //std::random_device generator;
    std::random_device seed;
    std::default_random_engine generator(seed());
    std::normal_distribution<double> distribution(mean, std_deviation);
    for (int i = 0; i < number_samples_; i++)
    {
        double reading = distribution(generator);

        // while (reading < min_range_ || reading > max_range_)
        // {
        //     reading = distribution(generator);
        // }
        if (reading < min_range_)
        {
            reading = min_range_;
        }
        else if (reading > max_range_)
        {
            reading = max_range_;
        }
        data_.push_back(reading);
        // data_[i] = reading;
    }
    return data_;
}
std::vector<double> Ranger::getData()
{
    return data_;
}
unsigned int Ranger::getAngularResolution(void)
{
    return angular_resolution_;
}

int Ranger::getOffset(void)
{
    return offset_;
}
unsigned int Ranger::getFieldOfView()
{
    return field_of_view_;
}

double Ranger::getMaxRange(void)
{
    return max_range_;
}
double Ranger::getMinRange(void)
{
    return min_range_;
}

SensingMethod Ranger::getSensingMethod(void)
{
    return sensing_type_;
}

std::string Ranger::getModel()
{
    return model_;
}
unsigned int Ranger::getNumberSamples(void)
{
    return number_samples_;
}

bool Ranger::setAngularResolution(unsigned int angular_resolution)
{
    if (angular_resolution >= 0)
    {
        angular_resolution_ = angular_resolution;
        return true;
    }
    else
    {
        return false;
    }
}
bool Ranger::setOffset(int offset)
{
    if (offset >= -360 && offset <= 360)
    {
        offset_ = offset;
        return true;
    }
    else
    {
        return false;
    }
}
bool Ranger::setFieldOfView(unsigned int field_of_view)
{
    if (field_of_view <= 180 || field_of_view >= -180)
    {
        field_of_view_ = field_of_view;
        return true;
    }
    else
    {
        return false;
    }
}

bool Ranger ::setNumberOfSamples(unsigned int samples)
{
    if (samples >= 0)
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
    if (model != " ")
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
    if (distance > 0)
    {
        max_range_ = distance;
        return true;
    }
    else
    {
        return false;
    }
}
bool Ranger ::setMinDistance(double distance)
{
    if (distance > 0)
    {
        min_range_ = distance;
        return true;
    }
    else
    {
        return false;
    }
}
SensingMethod Ranger::setSensingMethod(SensingMethod type)
{
    if (type == CONE || type == POINT)
        sensing_type_ = type;
}