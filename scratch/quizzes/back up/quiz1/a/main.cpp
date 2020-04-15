#include <iostream> // Includes std::cout and friends so we can output to console

int main (void) {
  //Create the person 'alice'
  Person alice;
  alice.name_ = "Alice";
  alice.age_ = 32;

  //Create the person 'bob' and print his age
  Person bob;
  bob.setName("Bob");

  if (bob.setAge(-62)) {
    std::cout << bob.getName() << "'s age is " << bob.getAge() << std::endl;
  } else {
    std::cout << "Something is wrong with " << bob.getName() << "'s age" << std::endl;
  }

  //Create the person 'carol'
  Person carol;
  carol.setName("Carol");
  carol.setAge(72);

  //! TODO Create a 'crowd' using a vector container of people


  //! TODO Create a function that greets the oldest crowd member
  //!
  //! The greeting should be: "Hello NAME, your our most distinguished member" << std::endl;


  return 0;
}
