#include <iostream>
#include <thread>
#include "radar.h"

int main (void){

  //! TASK 1:
  //! Instantiate radar object
  //! Display the default max distance
    Radar RadarObject;

    std::cout<<RadarObject.getMaxDistance()<<std::endl;
  //! TASK 2
  //! Create a thread tied to the spawn member function of Radar [radar.h](./a/dep/radar.h).
  //! Create a while loop in the main runs 50 times and displays the return value of getData member funcion.

    std::thread radar_thread(&Radar::getData, Radar());
    radar_thread.join();

    while(1){
        //radar_thread.
        //std::cout<<radar<<std::endl;


    }

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

  return 0;
}

