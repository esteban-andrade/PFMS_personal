#include <iostream>
#include <thread>
#include "radar.h"

int main(void)
{

  std::vector<Radar *> radars;
  radars.push_back(new Radar); // new instance
  //radars.push_back(new Radar);

  std::thread t1(&Radar::spawn, radars.at(0));
  //std::thread t2(&Radar::spawn, radars.at(1));
  unsigned int q1 = 0, q4 = 0;
  double refresh_rate, secondary_refresh;
  std::vector<std::thread> threads;
  for (auto radar : radars)
  {
    threads.push_back(std::thread(&Radar::spawn, radar)); // thread attach to spaw
  }
  double default_max_distance = radars.at(0)->getMaxDistance();
  std::cout << "Radar Default Max distance : " << default_max_distance << " m" << std::endl;
  auto start = std::chrono::steady_clock::now();
  while (q1 < 50)
  {
    std::vector<double> data = radars.at(0)->getData();
    auto end = std::chrono::steady_clock::now();
    auto difference = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    for (auto it : data)
    {
      std::cout << it << " ";
    }
    std::cout << std::endl;
    q1++;
    if (q1 == 50)
    {
      refresh_rate = difference.count() / 50;
      std::cout << "The Refresh Rate is :" << refresh_rate << " milliseconds for Default Config\n"
                << std::endl;
    }
  }

  auto start_4 = std::chrono::steady_clock::now();
  radars.at(0)->setMaxDistance(160); // set second config
  double max_distance = radars.at(0)->getMaxDistance();
  std::cout << "\nRadar second Config Max distance : " << max_distance << " m" << std::endl;
  while (q4 < 50) //secondary for-loop
  {

    std::vector<double> data = radars.at(0)->getData(); // get new data
    auto end = std::chrono::steady_clock::now();
    auto difference = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_4);
    for (auto it : data)
    {
      std::cout << it << " ";
    }
    std::cout << std::endl;
    q4++;
    if (q4 == 50)
    {
      secondary_refresh = difference.count() / 50;
      std::cout << "The Refresh Rate is : " << secondary_refresh << " milliseconds for secondary config\n"
                << std::endl;
    }
  }
  for (auto &t : threads)
  {
    t.join();
  }
  return 0;

  //! TASK 1:
  //! Instantiate radar object
  //! Display the default max distance

  // Radar radar;
  // double default_max_distance = radar.getMaxDistance();
  // std::cout << "Radar Default Max distance : " << default_max_distance << " m" << std::endl;

  // //! TASK 2
  // //! Create a thread tied to the spawn member function of Radar [radar.h](./a/dep/radar.h).
  // //! Create a while loop in the main runs 50 times and displays the return value of getData member funcion.
  // std::shared_ptr<Radar> radarPointer(new Radar());
  // // std::thread radar_thread(&Radar::spawn, radarPointer);
  // unsigned int q1 = 0, q4 = 0;
  // double refresh_rate_default;
  // double refresh_rate;
  // std::thread radar_thread(&Radar::spawn, radarPointer);
  // auto start = std::chrono::steady_clock::now();
  // while (q1 < 50)
  // {

  //   std::vector<double> data = radar.getData();
  //   auto end = std::chrono::steady_clock::now();
  //   auto difference = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  //   for (auto it : data)
  //   {
  //     std::cout << it << " ";
  //   }
  //   std::cout << std::endl;

  //   //std::cout << difference.count() << std::endl;
  //   q1++;
  //   if (q1 == 50)
  //   {
  //     refresh_rate_default = difference.count() / 50;
  //     std::cout << refresh_rate_default << " microseconds for Default Config" << std::endl;
  //   }
  // }
  // radar.setMaxDistance(160);
  // double max_distance = radar.getMaxDistance();
  // std::cout << "\nRadar Other Max distance : " << max_distance << " m" << std::endl;

  // auto start_4 = std::chrono::steady_clock::now();
  // while (q4 < 50)
  // {

  //   std::vector<double> data = radar.getData();
  //   auto end = std::chrono::steady_clock::now();
  //   auto difference = std::chrono::duration_cast<std::chrono::microseconds>(end - start_4);
  //   for (auto it : data)
  //   {
  //     std::cout << it << " ";
  //   }
  //   std::cout << std::endl;

  //   //std::cout << difference.count() << std::endl;
  //   q4++;
  //   if (q4 == 50)
  //   {
  //     refresh_rate = difference.count() / 50;
  //     std::cout << refresh_rate_default << " microseconds " << std::endl;
  //   }
  // }
  //! TASK 3: We were not provided a rate for the getData
  //! Using the chrono library and 50 sucsessive calls to getData in the while loop
  //! of your main you have already developed, compute the refresh rate (running rate)
  //! of getData (This will tell us the scanning time of the sensor).

  //! TASK 4: The scanning time is dependent on the MaxDistance.
  //! Add to your main code that sets the other supported MaxDistance and another
  //! while loop that queries getData another 50 times.
  //! Using the chrono library and these 50 sucsessive calls to getData in the while loop,
  //! compute the refresh rate (running rate) of getData (This will tell us the scanning time
  //! of the sensor in the other supported configuration).
  //radar_thread.join();
}
