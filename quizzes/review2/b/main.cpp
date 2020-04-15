// We need to include the declaration of our new car class in order to use it.
#include "car.h"

#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include<vector>

int main (void) {

    //Some specifications provided here, though you can use any of your own

    //Mercedes C180 Compressor.
    // height = 1.45 m, width = 1.77 m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg

    //Bugatti Veyron Super Sport.
    // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg


    //!TODO - TASK 2 : instatiate two objects of type `Car` with different specifications and determine's their top speed.
    // Create a Bugatti and Mercedes object
    Car bugatti("Bugatti", "Veyron", 1.19, 2.00, 1200, 0.35, 2200);
    Car merc("Mercedes", "C180 Compressor", 1.45, 1.77, 143, 0.29, 1200);
    std::cout << "Bugatti top speed is "<< bugatti.calculateTopSpeed() << std::endl;
    std::cout << "Merc top speed is " << merc.calculateTopSpeed() << std::endl << std::endl;

    bool still_racing = true;

    //Slow down the thread for 100 miscroseconds
    std::this_thread::sleep_for(std::chrono::microseconds(200));

    bugatti.calculateTopSpeed();
    merc.calculateTopSpeed();
    bool bugatti_topspeed = false;
    bool merc_topspeed = false;
    double time = 0;
    bool bugatti_decelerate = false;
    bool merc_decelerate = false;
    bool first_car = 0;
    bool bugatti_speed_zero = false;
    bool merc_speed_zero = false;
    double bugatti_speed0_time = 0;
    double merc_speed0_time = 0;



    //! Race until races
    std::cout << "Racing has commenced." << std::endl;
    while (still_racing){
        //!TODO - TASK 4 : Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and determines the time difference between the two vehicles reaching zero

        //std::cout << bugatti.getCurrentSpeed() << std::endl;
        //! Accelerate cars to top speed
        if(bugatti_topspeed == false)
        {
            bugatti.accelerate();
            if(bugatti.getCurrentSpeed() >= bugatti.calculateTopSpeed())
            {
                std::cout << "Buggatti reached top speed in " << time/1000000 << std::endl;
                bugatti_topspeed = true;
            }
        }

        if(merc_topspeed == false)
        {
            merc.accelerate();
            if(merc.getCurrentSpeed() >= (merc.calculateTopSpeed()))
            {
                std::cout << "Merc reached top speed in " << time/1000000 << std::endl;
                merc_topspeed = true;
            }
        }
        //std::cout << bugatti_topspeed << std::endl;
        //! Decelearate after reachig top speed to zero
        if(bugatti_topspeed == true && bugatti.getCurrentSpeed() >= 0)
        {
            bugatti.decelerate();
            bugatti_decelerate = true;
        }


        if(merc_topspeed == true && merc.getCurrentSpeed() >= 0)
        {
            merc.decelerate();
            merc_decelerate = true;
        }
        //std::cout << merc_topspeed << std::endl;

        //! Print when each car reach top speedand speed of Merc is " << merc.getCurrentSpeed()
        //! Print the car details of fisrt car to reach speed of zero
        if(first_car==0 && bugatti_decelerate == true)
        {
            std::cout << "Bugatti has reached top speed first" << std::endl;
            first_car = true;
        }
        if(first_car==0 && merc_decelerate == true)
        {
            std::cout << "Merc has reached top speed first" << std::endl;
            first_car = true;
        }
        //! Print the current speed of all other cars 
        if(bugatti_topspeed == true && bugatti.getCurrentSpeed()==0 && bugatti_speed_zero == false)
        {
            bugatti_speed0_time = time;
            std::cout << "Bugatti has reached speed 0 at time of " << time/1000000 << " seconds, and speed of Merc is " << merc.getCurrentSpeed() << std::endl;
            bugatti_speed_zero = true;
        }
        if(merc_topspeed == true && merc.getCurrentSpeed()==0 && merc_speed_zero == false)
        {
            merc_speed0_time = time;
            std::cout << "Merc has reached speed 0 at time of " << time/1000000 << " seconds and speed of Bugatti is " << bugatti.getCurrentSpeed() << std::endl;
            merc_speed_zero = true;
        }

        if(bugatti_speed_zero == true && merc_speed_zero == true)
        {
            if(bugatti_speed0_time >= merc_speed0_time)
            {
                std::cout << "Difference between times to reach 0 is " << (bugatti_speed0_time-merc_speed0_time)/1000000 << std::endl;
            }
            else
            {
                std::cout << "Difference between times to reach 0 is " << (merc_speed0_time-bugatti_speed0_time)/1000000 << std::endl;
            }
            still_racing = false;
        }


      //! NOTE: Keep the below sleep for the thread of 50 miscroseconds
      time += 50;
      std::this_thread::sleep_for(std::chrono::microseconds(50));

      }

    return 0;
}
