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
#include <mutex>

#include "simulator.h"
#include "control.h"

//For example purposes only, this thread gets the friendly aircraft's
//(red triangle) pose every 4 seconds. It plots this pose on the
//simulation (blue triangle) which stays on the image for 1 second, as per the
//'testPose()' documentation in the simualtor class.

void calculateThread(const std::shared_ptr<Simulator> &sim, Control &bogie)
{

  while (true)
  {
    bogie.calculate_bogiePos(sim);
    bogie.calculate_bogieOrien1(sim);
    bogie.calculate_bogieOrien2(sim);
    bogie.predict_futurePoint1(sim);
    bogie.predict_futurePoint2(sim);
    bogie.calculate_BogieFriendlyAngle1(sim);
    bogie.calculate_BogieFriendlyAngle2(sim);
    bogie.calculate_bogieVelocity(sim);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    bogie.getLastBogiePos(sim);
    bogie.getTime(sim);
  }
}

void printThread(const std::shared_ptr<Simulator> &sim, Control &bogie)
{
  while (true)
  {
    bogie.printscreen(sim);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void controlThread(const std::shared_ptr<Simulator> &sim, Control &bogie)
{
  while (true)
  {
    bogie.control_Friendly(sim);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

int main(void)
{
  std::vector<std::thread> threads;
  Control bogie;

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());

  threads.push_back(sim->spawn());

  threads.push_back(std::thread(calculateThread, sim, std::ref(bogie)));
  threads.push_back(std::thread(controlThread, sim, std::ref(bogie)));
  threads.push_back(std::thread(printThread, sim, std::ref(bogie)));
  //Join threads and begin!
  for (auto &t : threads)
  {
    t.join();
  }
  return 0;
}