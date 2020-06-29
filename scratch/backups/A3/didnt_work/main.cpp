
#include <thread>
#include <vector>
#include <iostream>
#include <deque>
#include <queue>
#include <condition_variable>
#include "datasynchronization.h"
#include "simulator.h"
#include "scanobjects.h"
#include "navigationaircraft.h"

void exampleThread(const std::shared_ptr<Simulator> &);
void controlThread(const std::shared_ptr<Simulator> &, std::condition_variable &, std::mutex &, AircraftContainer &,
                   std::vector<std::pair<long, long>> &, std::vector<std::pair<long, long>> &, DataSynchronization &);
void showData(const std::shared_ptr<Simulator> &, std::condition_variable &, std::condition_variable &,
              std::vector<RangeVelocityStamped> &, AircraftContainer &, std::vector<double> &, std::vector<double> &,
              std::vector<std::pair<long, long>> &, std::vector<std::pair<long, long>> &, DataSynchronization &);
double interpolate(long, long, double, double, double, bool);
void rangeBaseToBoggie(const std::shared_ptr<Simulator> &, std::condition_variable &, std::condition_variable &,
                       AircraftContainer &, std::vector<RangeVelocityStamped> &, DataSynchronization &);
void rangeFriendlyToBoggie(const std::shared_ptr<Simulator> &, std::condition_variable &, std::condition_variable &,
                           AircraftContainer &, std::vector<RangeBearingStamped> &, std::vector<double> &,
                           std::vector<double> &, DataSynchronization &);

int main(void)
{
  DataSynchronization data;
  std::vector<std::thread> threads;
  AircraftContainer friendly;
  std::mutex mtx;
  std::vector<std::pair<long, long>> base_compare;
  std::vector<std::pair<long, long>> friendly_compare;
  std::vector<RangeBearingStamped> friendly_data;
  std::vector<RangeVelocityStamped> base_data;
  std::vector<double> timer_friendly;
  std::vector<double> friendly_range;
  std::condition_variable cv;
  std::condition_variable cv_1;
  std::condition_variable cv_2;

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator());
  threads.push_back(sim->spawn());
  // threads.push_back(std::thread(controlThread, sim));
  // threads.push_back(std::thread(exampleThread, sim));
  threads.push_back(std::thread(controlThread, sim, std::ref(cv_1), std::ref(mtx), std::ref(friendly),
                                std::ref(base_compare), std::ref(friendly_compare), std::ref(data)));
  threads.push_back(std::thread(rangeFriendlyToBoggie, sim, std::ref(cv), std::ref(cv_2), std::ref(friendly),
                                std::ref(friendly_data), std::ref(timer_friendly), std::ref(friendly_range), std::ref(data)));
  threads.push_back(std::thread(rangeBaseToBoggie, sim, std::ref(cv), std::ref(cv_1), std::ref(friendly),
                                std::ref(base_data), std::ref(data)));
  threads.push_back(std::thread(showData, sim, std::ref(cv_1), std::ref(cv_2), std::ref(base_data), std::ref(friendly),
                                std::ref(timer_friendly), std::ref(friendly_range), std::ref(base_compare),
                                std::ref(friendly_compare), std::ref(data)));

  //Join threads and begin!
  for (auto &t : threads)
  {
    t.join();
  }

  return 0;
}
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
    sim->testPose(poses);

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
void controlThread(const std::shared_ptr<Simulator> &sim, std::condition_variable &cv_2, std::mutex &mtx, AircraftContainer &friendly,
                   std::vector<std::pair<long, long>> &base_comparison, std::vector<std::pair<long, long>> &friendly_comparison, DataSynchronization &data)
{
  ScanObjects scanner;
  NavigationAircraft navigate;
  Pose boggie_1, boggie_2;
  bool start = true;
  while (true)
  {
    Pose pose = sim->getFriendlyPose();
    std::vector<std::pair<long, long>> base = data.getBaseCompare();
    std::vector<std::pair<long, long>> friendly = data.getFrindlyCompare();
    std::vector<Pose> adjusted_boggie = scanner.getAnalysedBoggies();
    navigate.checkAirSpace(sim, pose);
    while (!base.empty() && !friendly.empty())
    {
      std::cout << "Synchronized Data "
                << " Base :" << base.front().first << " = " << base.front().second
                << " \nFriend: " << friendly.front().first << " = " << friendly.front().second << std::endl;
      scanner.circleIntersection(pose.position, sim->BSTATION_LOC, friendly.front().second, base.front().second, boggie_1, boggie_2);
      scanner.findReferencePosition(boggie_1, boggie_2);
      scanner.analyseBoggie(start, boggie_1, boggie_2);
      scanner.findCorrectedPosition(boggie_1, boggie_2);
      adjusted_boggie = scanner.getAnalysedBoggies();
      sim->testPose(adjusted_boggie);
      base = data.getBaseCompare();
      friendly = data.getFrindlyCompare();
      data.resetData();
    }
    navigate.getOrientation(adjusted_boggie, pose);
    navigate.aircraftNavigate();
    //Feed the watchdog control timer
    double lin = sim->getFriendlyLinearVelocity();
    double ang = sim->getFriendlyAngularVelocity();

    sim->controlFriendly(lin, ang);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void showData(const std::shared_ptr<Simulator> &sim, std::condition_variable &cv_1, std::condition_variable &cv_2, std::vector<RangeVelocityStamped> &base_data,
              AircraftContainer &friendly, std::vector<double> &timer_friendly, std::vector<double> &friendly_range, std::vector<std::pair<long, long>> &base_compare,
              std::vector<std::pair<long, long>> &friendly_compare, DataSynchronization &data)
{
  while (true)
  {
    std::unique_lock<std::mutex> locker(friendly.access);
    data.showData(locker);
  }
}

double interpolate(long xL, long xR, double yL, double yR, double x, bool extrapolate)
{
  if (!extrapolate)
  {
    if (x < xL)
    {
      yR = yL;
    }
    if (x > xR)
    {
      yL = yR;
    }
    double dydx = (yR - yL) / (xR - xL);
    return yL + dydx * (x - xL);
  }
}
void rangeBaseToBoggie(const std::shared_ptr<Simulator> &sim, std::condition_variable &cv, std::condition_variable &cv_1,
                       AircraftContainer &friendly, std::vector<RangeVelocityStamped> &base_data, DataSynchronization &data)
{
  std::vector<RangeVelocityStamped> range_base_to_boggie;
  while (true)
  {
    range_base_to_boggie = sim->rangeVelocityToBogiesFromBase();
    std::unique_lock<std::mutex> locker(friendly.access);
    data.getBaseData(range_base_to_boggie);
  }
}

void rangeFriendlyToBoggie(const std::shared_ptr<Simulator> &sim, std::condition_variable &cv, std::condition_variable &cv_2,
                           AircraftContainer &friendly, std::vector<RangeBearingStamped> &friendly_data, std::vector<double> &timer_friendly,
                           std::vector<double> &friendly_range, DataSynchronization &data)
{
  std::vector<RangeBearingStamped> range_friendly_to_boggie;
  while (true)
  {
    range_friendly_to_boggie = sim->rangeBearingToBogiesFromFriendly();
    std::unique_lock<std::mutex> locker(friendly.access);
    data.getFriendlyData(range_friendly_to_boggie);
    data.checkFriendlySize(locker);
  }
}
