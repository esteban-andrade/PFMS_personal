#include "person.h"
#include <iostream> // Includes std::cout and friends so we can output to console

#define _MAX_AGE 150 // define macro for max age

Person::Person() : Person("", 0) {}

Person::Person(std::string name, int age)
{
  name_ = name;
  age_ = age;
}

void Person::setName(std::string name)
{
  name_ = name;
}

bool Person::setAge(int age)
{
  if (age >= 0 && age < _MAX_AGE)
  {
    age_ = age;
  }
  else if (age >= _MAX_AGE)
  {
    age_ = 0;
  }
  else
  {
    age_ = 0;
  }
  return true;
}

std::string Person::getName(void)
{
  return name_;
}

int Person::getAge(void)
{
  return age_;
}
