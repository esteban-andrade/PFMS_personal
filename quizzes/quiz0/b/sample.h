
#ifndef __SAMPLE_H_
#define __SAMPLE_H_

#include <iostream> // Includes std::cout and friends so we can output to console
#include <cmath>    // Includes the math library

class Sample
{
public:
    Sample(double value);
    void setvalue(double value);
    double readvalue(void);

private:
    double value_;
};

#endif // __SAMPLE_H_
