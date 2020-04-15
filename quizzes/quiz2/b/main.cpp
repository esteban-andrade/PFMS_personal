// We need to include the declaration of our new car class in order to use it.
#include "car.h"

#include <iostream>
#include <thread> // std::this_thread::sleep_for
#include <vector>
#include <chrono>

int main(void)
{

  //Some specifications provided here, though you can use any of your own

  //Mercedes C180 Compressor.
  // height = 1.45 m, width = 1.77 m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg

  //Bugatti Veyron Super Sport.
  // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg

  //!TODO - TASK 2 : instatiate two objects of type `Car` with different specifications and determine's their top speed.
  // Car::Car(std::string make, std::string model,double height, double width,double horsePower, double dragCoefficient, double weight)
  // Create a Bugatti and Mercedes object
  // Car *Bugatti = new Car("Bugatti", "Veyron Super Sport", 1.19, 2.00, 1200, 0.35, 2200);
  // Car *Mercedez = new Car("Mercedes", "C180 Compressor", 1.45, 1.77, 143, 0.29, 1200);
  Car Bugatti("Bugatti", "Veyron Super Sport", 1.19, 2.00, 1200, 0.35, 2200);
  Car Mercedes("Mercedes", "C180 Compressor", 1.45, 1.77, 143, 0.29, 1200);
  bool still_racing = true;
  // std ::cout << "Bugatti top speed " << Bugatti->calculateTopSpeed() << std::endl;
  // std::cout << "Mercedes top speed " << Mercedez->calculateTopSpeed() << std::endl;

  std::vector<Car> cars;
  cars.push_back(Bugatti);
  cars.push_back(Mercedes);

  std::vector<double> top_speed;
  for (int i = 0; i < cars.size(); i++)
  {

    std::cout << cars.at(i).getMake() << " " << cars.at(i).getModel() << " top speed\t" << cars.at(i).calculateTopSpeed() << std::endl;
  }

  //Slow down the thread for 100 miscroseconds
  std::this_thread::sleep_for(std::chrono::microseconds(200));

  //! Race until races ends
  while (still_racing)
  {
    //!TODO - TASK 4 : Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and determines the time difference between the two vehicles reaching zero
    // //! Decelearate after reachig top speed to zero

    //! Print when each car reach top speed

    //! Print the car details of fisrt car to reach speed of zero
    //! Print the current speed of all other cars
    //  Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and determines the time difference between the two vehicles reaching zero.
    for (int i = 0; i < cars.size(); i++)
    {
      //cars.at(i).setStatus(0);
      if (cars.at(i).getCurrentSpeed() < cars.at(i).calculateTopSpeed() && cars.at(i).getStatus() == 0)
      {
        cars.at(i).accelerate();
        std::cout << cars.at(i).getMake() << " "
                  << " speed is " << cars.at(i).getCurrentSpeed() << std::endl;
      }
      else
      {
        if (cars.at(i).getCurrentSpeed() >= cars.at(i).calculateTopSpeed())
        {
          std::cout << cars.at(i).getMake() << " " << cars.at(i).getModel() << " "
                    << "has reached top speed" << std::endl;
          cars.at(i).setStatus(1);
        }
      }
    }
    for (int i = 0; i < cars.size(); i++)
    {

      if (cars.at(i).getCurrentSpeed() > 0 && cars.at(i).getStatus() == 1)
      {
        cars.at(i).decelerate();
        std::cout << cars.at(i).getMake() << " "
                  << " speed is " << cars.at(i).getCurrentSpeed() << std::endl;
      }
      else if (cars.at(i).getCurrentSpeed() == 0)
      {

        std::cout << cars.at(i).getMake() << " " << cars.at(i).getModel() << " "
                  << "has stop" << std::endl;
        cars.at(i).time;
      }
    }

    if (cars.at(0).getCurrentSpeed() == 0 && cars.at(1).getCurrentSpeed() == 0)
    {
      auto timespam = std::chrono::duration_cast<std::chrono::milliseconds>(cars.at(0).time - cars.at(1).time);
      std::cout << "The time difference between " << cars.at(0).getMake() << " and " << cars.at(1).getMake() << " is \n"
                << timespam.count() << " milliseconds" << std::endl;
      break;
    }

    //! NOTE: Keep the below sleep for the thread of 50 miscroseconds
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  return 0;
}

//! Accelerate cars to top speed
// std::cout << Bugatti.getMake() << " " << Bugatti.getModel() << "\t" << Bugatti.getCurrentSpeed() << std::endl;
// std::cout << Mercedes.getMake() << " " << Mercedes.getModel() << "\t" << Mercedes.getCurrentSpeed() << std::endl;
// if (Bugatti.getStatus())
// {
//   Bugatti.accelerate();
//   if (Bugatti.getCurrentSpeed() >= Bugatti.calculateTopSpeed())
//   {
//     std::cout << Bugatti.getMake() << " " << Bugatti.getModel() << " "
//               << "has reached top speed" << std::endl;
//     Bugatti.setStatus(0);
//   }
// }
// else if (!(Bugatti.getStatus()))
// {
//   Bugatti.decelerate();
//   if (Bugatti.getCurrentSpeed() <= 0)
//   {
//     Bugatti.getCurrentSpeed();
//     std::cout << Bugatti.getMake() << " " << Bugatti.getModel() << " "
//               << "has stopped" << std::endl;
//   }
// }

// if (Mercedes.getStatus())
// {
//   Mercedes.accelerate();
//   if (Mercedes.getCurrentSpeed() >= Mercedes.calculateTopSpeed())
//   {
//     std::cout << Mercedes.getMake() << " " << Mercedes.getModel() << " "
//               << "has reached top speed" << std::endl;
//     Mercedes.setStatus(0);
//   }
// }
// else if (!(Mercedes.getStatus()))
// {
//   Mercedes.decelerate();
//   if (Mercedes.getCurrentSpeed() <= 0)
//   {
//     Mercedes.getCurrentSpeed();
//     std::cout << Mercedes.getMake() << " " << Mercedes.getModel() << " "
//               << "has stopped" << std::endl;
//   }
// }

// for (int i = 0; i < cars.size(); i++)
//     {
//       std::cout << cars.at(i).getMake() << " "  << " speed is " << cars.at(i).getCurrentSpeed() << std::endl;
//       if (cars.at(i).getStatus())
//       {
//         cars.at(i).accelerate();
//         if (cars.at(i).getCurrentSpeed() >= top_speed.at(i))
//         {
//           std::cout << cars.at(i).getMake() << " " << cars.at(i).getModel() << " "
//                     << "has reached top speed" << std::endl;
//           cars.at(i).setStatus(0);
//         }
//       }
//       else if (!(cars.at(i).getStatus()))
//       {
//         cars.at(i).decelerate();
//         if (cars.at(i).getCurrentSpeed() <= 0.0)
//         {
//           std::cout << cars.at(i).getMake() << " " << cars.at(i).getModel() << " "
//                     << "has stop" << std::endl;
//                    cars.at(i).getCurrentSpeed();
//                    still_racing =false;
//                    break;
//         }
//       }

//       //! NOTE: Keep the below sleep for the thread of 50 miscroseconds
//       std::this_thread::sleep_for(std::chrono::microseconds(50));
//     }

// for (int i = 0; i < cars.size(); i++)
// {
//   //cars.at(i).setStatus(0);
//   if (cars.at(i).getCurrentSpeed() < cars.at(i).calculateTopSpeed() && cars.at(i).getStatus() == 0)
//   {
//     cars.at(i).accelerate();
//     std::cout << cars.at(i).getMake() << " "
//               << " speed is " << cars.at(i).getCurrentSpeed() << std::endl;
//   }
//   else
//   {
//     if (cars.at(i).getCurrentSpeed() >= cars.at(i).calculateTopSpeed())
//     {
//       cars.at(i).setStatus(1);
//     }
//   }
// }
// for (int i = 0; i < cars.size(); i++)
// {

//   if (cars.at(i).getCurrentSpeed() >0 && cars.at(i).getStatus() == 1)
//   {
//     cars.at(i).decelerate();
//     std::cout << cars.at(i).getMake() << " "
//               << " speed is " << cars.at(i).getCurrentSpeed() << std::endl;
//   }

// }
