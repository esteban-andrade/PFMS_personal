
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"
#include "Navigation.h"
#include "Scanner.h"
#include "DataSynch.h"
#include <condition_variable>

//For example purposes only, this thread attmps to get the friendly aircraft's
//(red triangle) pose every 4 seconds. It plots this pose on the
//simulation (blue triangle) which stays on the image for 1 second, as per the
//'testPose()' documentation in the simualtor class.
void exampleThread(const std::shared_ptr<Simulator> &sim)
{
  while (true)
  {
    //Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    std::vector<Pose> poses;
    poses.push_back(pose);
    //sim->testPose(poses);

    std::cout << "[" << sim->elapsed() / 1000 << "s]" << std::endl;
    std::cout << "Friendly {x, y, orientation}:" << std::endl;
    std::cout << "  - x: " << pose.position.x << "m" << std::endl;
    std::cout << "  - y: " << pose.position.y << "m" << std::endl;
    std::cout << "  - orient: " << pose.orientation * 180 / M_PI << " deg" << std::endl
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

//This thread will simply get the current velocity and feed it back into
//controlled velocity, at the designated minimum time required (watchdog time) refer
//'controlFriendly()' documentation in the simualtor class.
void controlThread(const std::shared_ptr<Simulator> &sim, Navigation &navigation, Scanner &scan, DataSynch &data)
{
  data.controlThread(sim, navigation, scan, data);
}
void analysisThread(const std::shared_ptr<Simulator> &sim, Navigation &navigation, Scanner &scan, Pose &friendly, DataSynch &data)
{
  data.dataThread(navigation, sim, friendly);
}
void processThread(const std::shared_ptr<Simulator> &sim, Navigation &navigation, Scanner &scan, DataSynch &data)
{
  data.processThread(scan);
}
int main(void)
{
  std::vector<std::thread> threads;
  Navigation navigate;
  Scanner scan;
  DataSynch data;
  Pose friendly;

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());
  threads.push_back(sim->spawn());

  //threads.push_back(std::thread(exampleThread, sim));
  // threads.push_back(std::thread(analysisThread, sim));
  //threads.push_back(std::thread(controlThread, sim));
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
