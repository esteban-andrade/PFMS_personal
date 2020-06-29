#include "laser.h"

// laser specifications
static const int laser_field_of_View = 180;
static const double laser_max_range = 8.0;
static const double laser_min_range = 0.2;
const std::string laser_model = "SICK-XL";
static const int laser_default_angular_resolution = 10;
static const int laser_secondary_angular_resolution = 30;
static const double laser_default_offset = 0;
static const unsigned int NumberOfLaserConfigurations = 2;
static const int number_samples_default = (laser_field_of_View / laser_default_angular_resolution) + 1;
static const int number_samples_secondary = (laser_field_of_View / laser_secondary_angular_resolution) + 1;
SensingMethod laser_sensing_method = POINT;

/*
Laser default constructor
Will set laser specs to default
*/
Laser::Laser()
{
    setAngularResolution(laser_default_angular_resolution);
    setOffset(laser_default_offset);
    setFieldOfView(laser_field_of_View);
    setModel(laser_model);
    setMaxDistance(laser_max_range);
    setMinDistance(laser_min_range);
    setNumberOfSamples(number_samples_default);
    setSensingMethod(laser_sensing_method);
    setLaserConfiguration(1);
}

//Constructor that will set an specific laser configuration
Laser::Laser(int config)
{
    setOffset(laser_default_offset);
    setFieldOfView(laser_field_of_View);
    setModel(laser_model);
    setMaxDistance(laser_max_range);
    setMinDistance(laser_min_range);
    setSensingMethod(laser_sensing_method);
    setLaserConfiguration(config);
}

// will set the lasert angular reaolution
bool Laser::setAngularResolution(double angular_resolution)
{
    // will perform sanity check in the input to ensure adequate inputs
    if (angular_resolution == laser_default_angular_resolution || angular_resolution == laser_secondary_angular_resolution)
    {
        angular_resolution_ = angular_resolution;
        return true;
    }
    else
    {
        return false;
    }
}

// will set the an specific laser configuration 
bool Laser::setLaserConfiguration(int config)
{
    if (config == 1)
    {
        laser_configuration = 1;
        setNumberOfSamples(number_samples_default);
        setAngularResolution(laser_default_angular_resolution);
        return true;
    }
    else if (config == 2)
    {
        laser_configuration = 2;
        setNumberOfSamples(number_samples_secondary);
        setAngularResolution(laser_secondary_angular_resolution);
        return true;
    }
    else
    {
        return false;
    }
}

// return laser configuration
unsigned int Laser::getLaserConfiguration(void)
{
    return laser_configuration;
}

//returns number of valid laser configuration
unsigned int Laser::getNumberOfLaserConfigurations(void)
{
    return NumberOfLaserConfigurations;
}
