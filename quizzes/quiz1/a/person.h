#ifndef PERSON_H // An 'include guard' to prevent double declaration of any identifiers in this library
#define PERSON_H

#include <string>

class Person
{
public:
  Person();                          // default contructor
  Person(std::string name, int age); // constructor for name and age
  void setName(std::string name);
  bool setAge(int age);

  std::string getName(void);
  int getAge(void);

private:
  std::string name_;
  int age_;
};

#endif // PERSON_H
