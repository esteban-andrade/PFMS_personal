#include "person.h"
#include <iostream> // Includes std::cout and friends so we can output to console


void Person::setName(std::string name) {
  name_ = name;
}

bool Person::setAge(int age) {
  age_ = age;
  return true;
}

std::string Person::getName(void) {
  return name_;
}

int Person::getAge(void) {
  return age_;
}
