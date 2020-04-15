#include <iostream> // Includes std::cout and friends so we can output to console
#include <vector>
#include "person.h"

int main (void) {
  //Create the person 'alice'
  Person alice;
  alice.setName("Alice");
  //Age Check
  if (alice.setAge(32)) {
    std::cout << alice.getName() << "'s age is " << alice.getAge() << std::endl;
  } else {
    std::cout << "Something is wrong with " << alice.getName() << "'s age" << std::endl;
  }

  //Create the person 'bob' and print his age
  Person bob;
  bob.setName("Bob");
  //Age Check
  if (bob.setAge(-62)) {
    std::cout << bob.getName() << "'s age is " << bob.getAge() << std::endl;
  } else {
    std::cout << "Something is wrong with " << bob.getName() << "'s age" << std::endl;
  }

  //Create the person 'carol'
  Person carol;
  carol.setName("Carol");
  //Age Check
  if (carol.setAge(72)) {
    std::cout << carol.getName() << "'s age is " << carol.getAge() << std::endl;
  } else {
    std::cout << "Something is wrong with " << carol.getName() << "'s age" << std::endl;
  }

  //! TODO Create a 'crowd' using a vector container of people
  std::vector<Person> crowd;
  crowd.clear();
  crowd.push_back(alice);
  crowd.push_back(bob);
  crowd.push_back(carol);

  //! TODO Create a function that greets the oldest crowd member
  int age_oldest=0, oldest_member,i,j;
  for(i=0;i<3;i++){
      if(crowd[i].getAge()>age_oldest){
          age_oldest=crowd[i].getAge();
      }
  }
  for(j=0;j<3;j++){
      if(age_oldest==crowd[j].getAge()){
          oldest_member=j;
      }
  }

  //! The greeting should be: "Hello NAME, your our most distinguished member" << std::endl;
  std::cout<<"Hello "<<crowd[oldest_member].getName()<<", your our most distinguished member"<<std::endl;

  return 0;
}

//Final version
