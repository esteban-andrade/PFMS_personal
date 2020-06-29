
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"
#include "Navigation.h"
#include "Scanner.h"

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
void controlThread(const std::shared_ptr<Simulator> &sim)
{

  while (true)
  {
    Navigation navigate;
    Pose pose = sim->getFriendlyPose();
    //Feed the watchdog control timer
    navigate.getOrientation(sim, pose);
    navigate.navigate(sim, pose);
    double distance = navigate.getDistanceFromBase(pose);
    //double lin = sim->getFriendlyLinearVelocity();
    //double ang = sim->getFriendlyAngularVelocity();

    //sim->controlFriendly(lin, ang);
    //std::cout << "Distance" << distance << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
void analysisThread(const std::shared_ptr<Simulator> &sim)
{
  Scanner scan;
  scan.baseScan(sim);
  scan.airCraftScan(sim);
  scan.friendlyRelariveBase(sim);
  scan.determineTarget(sim);
  int i = 0;

  while (true)
  {

    scan.determineTarget(sim);
    scan.baseScan(sim);
    scan.airCraftScan(sim);
    scan.friendlyRelariveBase(sim);
    scan.predictTarget(sim);
    Pose pose;
    std::vector<Pose> poses;
    pose.position.x = scan.getTarget().at(i).first;
    pose.position.y = scan.getTarget().at(i).second;
    poses.push_back(pose);
    sim->testPose(poses);

    // std::cout << i << ") Distance from base" << scan.getBaseScan().at(i).first << std::endl;
    // std::cout << i << ") Distance from friendly" << scan.getFriendlyScan().at(i).first << std::endl;
    // std::cout << i << ") Distance friendly from base" << scan.getBaseToFriendly().at(i).first << std::endl;
    std::cout << "Bogie Target :" << scan.getBogieIndex() << std::endl;
    std::cout << "x:" << scan.getTarget().at(i).first << " y " << scan.getTarget().at(i).second << std::endl;
    std::cout << " predicted x:" << scan.getPredictedTarget().at(i).first << " predicted y " << scan.getPredictedTarget().at(i).second << std::endl;
    i++;
    if (i > 3)
    {
      i = 0;
      scan.resetData();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
int main(void)
{
  std::vector<std::thread> threads;

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());
  threads.push_back(sim->spawn());
  threads.push_back(std::thread(controlThread, sim));
  threads.push_back(std::thread(exampleThread, sim));
  threads.push_back(std::thread(analysisThread, sim));
  //Join threads and begin!
  for (auto &t : threads)
  {
    t.join();
  }

  return 0;
}
