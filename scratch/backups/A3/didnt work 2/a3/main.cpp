
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"
#include "Navigation.h"
#include "Scanner.h"
#include <condition_variable>
#include "Navigate.h"
#include "Seeker.h"
#include "DataSynch.h"

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
// void controlThread(const std::shared_ptr<Simulator> &sim)
// {

//   while (true)
//   {
//     Navigation navigate;
//     Pose pose = sim->getFriendlyPose();
//     //Feed the watchdog control timer
//     navigate.getOrientation(sim, pose);
//     //navigate.navigate(sim, pose);
//     double distance = navigate.getDistanceFromBase(pose);
//     double lin = sim->getFriendlyLinearVelocity();
//     double ang = sim->getFriendlyAngularVelocity();

//     sim->controlFriendly(lin, ang);
//     //std::cout << "Distance" << distance << std::endl;
//     std::this_thread::sleep_for(std::chrono::milliseconds(10));
//   }
// }
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

void controlThread(const std::shared_ptr<Simulator> &sim, std::condition_variable &cv2, std::mutex &mtx, AircraftContainer &friendly, std::vector<double> &baseCompare, std::vector<double> &friendCompare, DataSynch &data)
{
  Seeker seek;
  Navigate navigate;
  Pose bogie0;
  Pose bogie1;
  bool start = true;
  while (true)
  {
    Pose friendly = sim->getFriendlyPose();
    std::vector<double> base = data.getBaseCompare();
    std::vector<double> friendlyData = data.getFriendlyCompare();
    std::vector<Pose> correctBogie = seek.getPredictedBogie();
    navigate.checkAirspace(sim, friendly);
    while (!base.empty() && !friendlyData.empty())
    {
      seek.circleIntersection(friendly.position, sim->BSTATION_LOC, friendlyData.at(1), base.at(1), bogie0, bogie1);
      seek.findPosition(bogie0, bogie1);
      seek.analyseBogie(start, bogie0, bogie1);
      seek.getPosition(bogie0, bogie1);
      correctBogie = seek.getPredictedBogie();
      sim->testPose(correctBogie);
      base = data.getBaseCompare();
      friendlyData = data.getFriendlyCompare();
      data.resetData();
    }
    navigate.analyseOrientation(correctBogie, friendly);
    navigate.navigateAircraft();
    double lin = sim->getFriendlyLinearVelocity();
    double ang = sim->getFriendlyAngularVelocity();

    sim->controlFriendly(lin, ang);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
void showData(const std::shared_ptr<Simulator> &sim, std::condition_variable &cv1, std::condition_variable &cv2, std::vector<RangeVelocityStamped> &baseContainer, AircraftContainer &friendly, std::vector<double> &timeFriend, std::vector<double> &rangeFriend, std::vector<double> &baseCompare, std::vector<double> &friendCompare, DataSynch &data)
{

  while (true)
  {
    std::unique_lock<std::mutex> locker(friendly.access);
    data.showData(locker);
  }
}
double interpolate(double xL, double xR, double yL, double yR, double x, bool extrapolate)
{
  if (!extrapolate)
  {
    if (x < xL)
      yR = yL;
    if (x > xR)
      yL = yR;
  }
  double dydx = (yR - yL) / (xR - xL);
  return yL + dydx * (x - xL);
}
void rangeBaseToBogie(const std::shared_ptr<Simulator> &sim, std::condition_variable &cv, std::condition_variable &cv1, AircraftContainer &friendly, std::vector<RangeVelocityStamped> &baseContainer, DataSynch &data)
{
  std::vector<RangeVelocityStamped> base_to_bogie;
  while (true)
  {
    base_to_bogie = sim->rangeVelocityToBogiesFromBase();
    std::unique_lock<std::mutex> locker(friendly.access);
    data.acquireBaseData(base_to_bogie);
  }
}

void rangeFriendlyToBogie(const std::shared_ptr<Simulator> &sim, std::condition_variable &cv, std::condition_variable &cv2, AircraftContainer &friendly, std::vector<RangeBearingStamped> &friendlyContainer, std::vector<double> &timeFriend, std::vector<double> &rangeFriend, DataSynch &data)
{
  std::vector<RangeBearingStamped> friendly_to_bogie;
  while (true)
  {
    friendly_to_bogie = sim->rangeBearingToBogiesFromFriendly();
    std::unique_lock<std::mutex> locker(friendly.access);
    data.acquireFriendlyData(friendly_to_bogie);
    data.checkFriendlySize(locker);
  }
}
int main(void)
{
  std::vector<std::thread> threads;
  DataSynch data;
  AircraftContainer friendly;
  std::mutex mtx;
  std::vector<double> baseCompare;
  std::vector<double> friendCompare;
  std::deque<RangeBearingStamped> friendlyContainer;
  std::vector<double> timeFriend;
  std::vector<double> rangeFriend;
  std::deque<RangeVelocityStamped> baseContainer;

  std::condition_variable cv;
  std::condition_variable cv1;
  std::condition_variable cv2;
  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());
  threads.push_back(sim->spawn());
  //threads.push_back(std::thread(controlThread, sim));
  // threads.push_back(std::thread(exampleThread, sim));
  //threads.push_back(std::thread(analysisThread, sim));
  //Join threads and begin!
  threads.push_back(std::thread(controlThread, sim, std::ref(cv1), std::ref(mtx), std::ref(friendly), std::ref(baseCompare), std::ref(friendCompare), std::ref(data)));
  //threads.push_back(std::thread(rangeFriendlyToBogie, sim, std::ref(cv), std::ref(cv2), std::ref(friendly), std::ref(friendlyContainer), std::ref(timeFriend), std::ref(rangeFriend), std::ref(data)));
  // threads.push_back(std::thread(rangeBaseToBogie, sim, std::ref(cv), std::ref(cv1), std::ref(friendly), std::ref(baseContainer), std::ref(data)));
  // threads.push_back(std::thread(showData, sim, std::ref(cv1), std::ref(cv2), std::ref(baseContainer), std::ref(friendly), std::ref(timeFriend), std::ref(rangeFriend), std::ref(baseCompare), std::ref(friendCompare), std::ref(data)));
  for (auto &t : threads)
  {
    t.join();
  }

  return 0;
}
