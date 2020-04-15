#include "sample.h"


Sample::Sample(double value){
    value_ = value;
}

double Sample::readvalue    (){
    return value_;
}

void Sample::setvalue(double value){
    value_ = value;
}
