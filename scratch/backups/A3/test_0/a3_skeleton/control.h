#ifndef CONTROL_H
#define CONTROL_H
#include <thread>
#include <vector>
#include <iostream>
#include <condition_variable>
#include <mutex>

#include "simulator.h"

class Control
{
public:
    Control();
    //! Calculates the two possible bogie locations and stores in the BogiePos vector
    void calculate_bogiePos(const std::shared_ptr<Simulator> &sim);
    //! Calculates the first potential bogie orientation
    void calculate_bogieOrien1(const std::shared_ptr<Simulator> &sim);
    //! Calculates the second potential bogie orientation
    void calculate_bogieOrien2(const std::shared_ptr<Simulator> &sim);
    //! Calculates the first potential angle from friendly aircraft to bogie
    void calculate_BogieFriendlyAngle1(const std::shared_ptr<Simulator> &sim);
    //! Calculates the second potential angle from friendly aircraft to bofie
    void calculate_BogieFriendlyAngle2(const std::shared_ptr<Simulator> &sim);
    //! Calculates the two possible bogie velocities
    void calculate_bogieVelocity(const std::shared_ptr<Simulator> &sim);
    //! Predicts the potential location of one bogie in the future based off certain variables
    void predict_futurePoint1(const std::shared_ptr<Simulator> &sim);
    //! Predicts the potential location of one bogie in the future based off certain variables
    void predict_futurePoint2(const std::shared_ptr<Simulator> &sim);
    //! Gets the previous bogie position
    void getLastBogiePos(const std::shared_ptr<Simulator> &sim);
    //! Gets the time of the previous bogie position
    void getTime(const std::shared_ptr<Simulator> &sim);
    //! Algorithm for following bogie position 1
    void followBogie1(const std::shared_ptr<Simulator> &sim);
    //! Algorithm for following bogie position 2
    void followBogie2(const std::shared_ptr<Simulator> &sim);
    //! Prints the information on the screen
    void printscreen(const std::shared_ptr<Simulator> &sim);
    //! Selects whether to follow bogie position 1 or 2
    void control_Friendly(const std::shared_ptr<Simulator> &sim);

private:
    std::vector<double> RangeBogieBase{1};

    std::vector<Pose> LastBogiePos{2, {0, 0}}; //!< Used to store the previous bogie positions
    std::vector<Pose> BogiePos{2, {0, 0}};     //!< Used to store the bogie positions
    std::vector<Pose> PredictPos{2, {0, 0}};   //!< Used to store the predicted bogie positions
    std::vector<long> LastTime{1};             //!< Used to store the time for the previous bogie positions

    std::mutex mtx; //!< Mutex used to protect shared data from being simultaneously accessed by other threads
    std::condition_variable cv;

    double theta1;                  //!< stores the angle from bogie to friendly for bogie 1
    double theta2;                  //!< stores the angle from bogie to friendly for bogie 2
    double distance_bogie_friendly; //!< stores the range from bogie to friendly
    double distance_bogie_base;     //!< stores the range from bogie to base

    double velocity1; //!< stores the velocity for bogie 1
    double velocity2; //!< stores the velocity for bogie 2

    double Difference; //!< used to check how much the aircraft needs to turn to face the bogie
    //double counter;
    long timeBogie;     //!< the current time
    double number = 50; //!< the number used to calculate the predicted coordinates
};
#endif // CONTROL_H