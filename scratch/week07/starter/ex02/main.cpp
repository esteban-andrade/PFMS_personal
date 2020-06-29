#include <iostream>
#include <thread>
#include "radar.h"

int main(void)
{

  // instantiate radar object
  // Radar radar;

  // while (true){
  //   std::vector <double> data = radar.getData();

  //   for(auto elem : data){
  //     std::cout << elem << " ";
  //   }
  //   std::cout << std::endl;
  // }

  std::vector<Radar *> radars;
  radars.push_back(new Radar);
  radars.push_back(new Radar);
  radars.push_back(new Radar);

  std::thread t1(&Radar::start, radars.at(0));
  std::thread t2(&Radar::start, radars.at(1));
  std::thread t3(&Radar::start, radars.at(2));

  std::vector<std::thread> threads;

  for (auto radar : radars)
  {
    threads.push_back(std::thread(&Radar::start, radar));
  }
  std::vector<double> data = radars.at(0)->getData();
  for (auto elem : data)
  {
    std::cout << elem << " ";
  }
  std::cout << std::endl;

  for (auto &t : threads)
  {
    t.join();
  }
  return 0;
}
