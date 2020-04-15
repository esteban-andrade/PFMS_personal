#include "person.h"
#include <iostream> // Includes std::cout and friends so we can output to console

Person::Person(int age_1){
    age_=age_1;
}

void Person::setName(std::string name) {
  name_ = name;
}

bool Person::setAge(int age) {
  if(age<0){age=-age;}
  age_ = age;
  if(age_<122)
    {return true;}
  else
    {return false;}
}

std::string Person::getName(void) {
  return name_;
}

int Person::getAge(void) {
  return age_;
}

//Final version
