#include "sonar.h"

static const int sonar_field_of_view = 20;
static const double sonar_max_distance = 16.0;
static const double sonar_min_distance = 0.2;
static const std::string sonar_model = "SN-001";
static const double sonar_default_offset = 0;
static const int sonar_angular_resolution = 20;
static const int number_samples = sonar_field_of_view/sonar_angular_resolution;
SensingMethod sonar_sensing_method = CONE;

Sonar::Sonar()
{
    setAngularResolution(sonar_angular_resolution);
    setOffset(sonar_default_offset);
    setFieldOfView(sonar_field_of_view);
    setModel(sonar_model);
    setMaxDistance(sonar_max_distance);
    setMinDistance(sonar_min_distance);
    setNumberOfSamples(number_samples);
    setSensingMethod(sonar_sensing_method);
}
