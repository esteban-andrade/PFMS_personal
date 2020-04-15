#include "person.h" // include header file

#include <iostream> // Includes std::cout and friends so we can output to console
#include <vector>

#define _MAX_AGE 150 // define macro for max possible age

// function declaration
void oldestMember(std::vector<Person> &vec);

int main(void)
{
  //Create the person 'alice'
  Person alice;
  alice.setName("Alice");
  alice.setAge(32);

  //Create the person 'bob' and print his age
  Person bob;
  bob.setName("Bob");
  bob.setAge(62);

  //Create the person 'carol'
  Person carol;
  carol.setName("Carol");
  carol.setAge(72);

  //! TODO Create a 'crowd' using a vector container of people
  std::vector<Person> crowrd;
  crowrd.push_back(alice);
  crowrd.push_back(bob);
  crowrd.push_back(alice);

  //creation of objects
  crowrd.push_back(Person("Carlos", 15));
  crowrd.push_back(Person("John", 65));
  crowrd.push_back(Person("Mary", 85));

  //! TODO Create a function that greets the oldest crowd member
  oldestMember(crowrd);
  //! The greeting should be: "Hello NAME, your our most distinguished member" << std::endl;

  return 0;
}

void oldestMember(std::vector<Person> &vec)
{
  int highest_age = 0;
  std::string name;
  for (auto age : vec)
  {
    if (age.getAge() > highest_age)
    {
      highest_age = age.getAge();
      name = age.getName();
    }
  }
  std::cout << "Hello " << name << " ,your our most distinguished member" << std::endl;
}