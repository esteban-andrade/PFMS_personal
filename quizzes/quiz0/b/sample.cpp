
#include "sample.h"

Sample::Sample(double value)
{
     value_ = value;  // this is used with the constructor to assign the passed value to value_
}

void Sample::setvalue(double value)
{
    value_ = value; // this was used by passing the value with a method and assigning the value to value_
}

double Sample::readvalue(void)
{
    return value_; // returns the assigned value 
}