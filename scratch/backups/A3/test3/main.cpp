/*! @file
 *
 *  @brief Main entry point for assignment 3.
 *
 *  TODO: Add information here
 *
 *  @author {TODO: Your student name + id}
 *  @date {TODO}
*/
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"
#include "intercept_bogie.h"

int main(void)
{
  std::vector<std::thread> threads;

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());
  std::shared_ptr<InterceptBogies> IB(new InterceptBogies(std::ref(sim)));
  // std::shared_ptr<Friendly> friendly(new Friendly(std::ref(sim)));
  // std::shared_ptr<Bogie> bogie(new Bogie(std::ref(sim)));

  threads.push_back(sim->spawn());

  threads.push_back(std::thread(&InterceptBogies::controlThread, IB));  //control thread
  threads.push_back(std::thread(&InterceptBogies::dataThread, IB));  //data thread
  threads.push_back(std::thread(&InterceptBogies::processThread, IB));  //process thread

  //Join threads and begin!
  for(auto & t: threads){
    t.join();
  }

  return 0;
}
