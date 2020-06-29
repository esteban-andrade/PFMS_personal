
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"
#include "Navigation.h"
#include "Scanner.h"
#include "DataSynch.h"
#include <condition_variable>

void controlThread(const std::shared_ptr<Simulator> &sim, Navigation &navigation, Scanner &scan, DataSynch &data) // control function with thread
{
  data.controlThread(sim, navigation, scan, data); // data synchronization control thread
}
void analysisThread(const std::shared_ptr<Simulator> &sim, Navigation &navigation, Scanner &scan, Pose &friendly, DataSynch &data) // function to analyse data in thread
{
  data.dataThread(navigation, sim, friendly); // call data threads inside data synchronization
}
void processThread(const std::shared_ptr<Simulator> &sim, Navigation &navigation, Scanner &scan, DataSynch &data) // function to process the data in the thread
{
  data.processThread(scan); // process all the data with process method inside data synchronization
}
int main(void)
{
  std::vector<std::thread> threads; // create vector of threads
  Navigation navigate;              // create navigate object
  Scanner scan;                     // create Scanner object
  DataSynch data;                   // create Datasynch object
  Pose friendly;                    // create Pose frienly object

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());
  threads.push_back(sim->spawn());

  //call functions and threads and pass reference objects
  threads.push_back(std::thread(controlThread, sim, std::ref(navigate), std::ref(scan), std::ref(data)));
  threads.push_back(std::thread(analysisThread, sim, std::ref(navigate), std::ref(scan), std::ref(friendly), std::ref(data)));
  threads.push_back(std::thread(processThread, sim, std::ref(navigate), std::ref(scan), std::ref(data)));
  //Join threads and begin!

  for (auto &t : threads)
  {
    t.join();
  }

  return 0;
}
