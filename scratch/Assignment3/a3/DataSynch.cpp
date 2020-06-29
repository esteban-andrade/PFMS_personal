#include "DataSynch.h"
#include <vector>
#include <mutex>
#include <condition_variable>

//debugger
//#define DEBUG 1

DataSynch::DataSynch()
{
}

DataSynch::~DataSynch()
{
}
void DataSynch::dataThread(Navigation &navigate, const std::shared_ptr<Simulator> &sim, Pose friendly)
{
    std::unique_lock<std::mutex> locker(mtx); // unique lock with mutex
    while (true)
    {
        friendly_simulator_ = sim->getFriendlyPose();     // get data from the simulator
        navigate.navigate(friendly_simulator_, friendly); // engage navigation of the enviroment
        navigate.purePursuit(closest_bogie_, friendly);   // engage purepursuit
        double lin_vel = navigate.getLinearVelocity();    // get linear velocity
        double ang_vel = navigate.getAngularSpeed();      // get angular velocity
        sim->controlFriendly(lin_vel, ang_vel);           // control the friendly
        mtx.unlock();                                     // unlock thread
    }
}

void DataSynch::controlThread(const std::shared_ptr<Simulator> &sim, Navigation &navigate, Scanner &scan, DataSynch &data)
{
    int i = 0; // variables to be used along with the debugger

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        base_scan_sim_ = sim->rangeVelocityToBogiesFromBase();            // get data in real time from the base
        friendly_scan_results_ = sim->rangeBearingToBogiesFromFriendly(); // get friendly data in real time
        pose_simulation_ = sim->getFriendlyPose();                        // get the friendly pose in real time
        std::unique_lock<std::mutex> locker(mtx);                         // lock thread for analysis
        scan.baseScan(base_scan_sim_);                                    // pass base can
        scan.airCraftScan(friendly_scan_results_);                        // pass aircraft scan
        std::this_thread::sleep_for(std::chrono::milliseconds(10));       // unavoidable wait to ensure positions are stored
        scan_prediction_ = sim->rangeBearingToBogiesFromFriendly();       // get second data from the bogie pose
        scan.determineTarget(pose_simulation_, friendly_scan_results_);   // pass data in order to determine target
        scan.friendlyRelativeBase(pose_simulation_);                      // pass data to get data of friendly relative to base
        scan.predictTarget(friendly_scan_results_, scan_prediction_);     // pass data to predict the target
        pose_.position.x = scan.getPredictedTarget().at(i).first;         // store pose x position
        pose_.position.y = scan.getPredictedTarget().at(i).second;        // store pose y position
        pose_.orientation = scan.getOrientationPrediction();              // store pose orientation
        poses_.push_back(pose_);                                          // push back data into vector of poses
        sim->testPose(poses_);                                            // render data

//std::cout << " Distance from base " << scan.getBaseScan().at(i).first << std::endl;
#ifdef DEBUG
        std::cout << "\n[" << sim->elapsed() / 1000 << "s]" << std::endl;
        std::cout << "Friendly {x, y, orientation}:" << std::endl;
        std::cout << "  - x: " << navigate.getCurrentPose().position.x << "m" << std::endl;
        std::cout << "  - y: " << navigate.getCurrentPose().position.y << "m" << std::endl;
        std::cout << "  - orient: " << navigate.getCurrentPose().orientation * 180 / M_PI << " deg" << std::endl;
        std::cout << " Distance from friendly " << scan.getFriendlyScan().at(0).first << std::endl;
        std::cout << " Distance friendly from base " << scan.getBaseToFriendly().at(0).first << std::endl;
        std::cout << " target x:" << scan.getTarget().at(i).first << " target y " << scan.getTarget().at(i).second << std::endl;
        std::cout << " Predicted x:" << scan.getPredictedTarget().at(0).first << " Predicted y " << scan.getPredictedTarget().at(0).second << std::endl;
        std::cout << " Bearing Angle :" << scan.getBearingAngle() << std::endl;
        // std::cout << " Bogie orientation " << scan.getBaseToFriendly().at(0).second << "\tBogie range " << scan.getBaseToFriendly().at(i).first << std::endl;
        std::cout << " Vx " << scan.getVelocityVector().at(0).first << "\tVy " << scan.getBaseToFriendly().at(0).second << " Velocity Scalar " << scan.getObjectVelocity().at(0) << "\n"
                  << std::endl;
#endif
        i++; // value used to increase value that will be used in vectors
        if (i > 3)
        {
            i = 0;            // reset variable
            scan.resetData(); //reset data
        }
        process_ready_ = true; // set process as ready
        mtx.unlock();          //unlock thread
        cv.notify_all();       //notify thread
        poses_.clear();        // clear vector posesin order to have a single prediction
    }
}

void DataSynch::processThread(Scanner &scanner)
{
    while (true)
    {
        std::unique_lock<std::mutex> locker(mtx);                                                               // lock thread
        cv.wait(locker, [&] { return (process_ready_); });                                                      //conditinal variable with lamda function
        bogies = scanner.getFriendlyScanResults();                                                              // get data fron friendly
        sort(bogies.begin(), bogies.end(), [](const RangeBearingStamped &lhs, const RangeBearingStamped &rhs) { // sort data from friendly
            return lhs.range < rhs.range;
        });

        closest_bogie_ = bogies.at(0); // store most adequate bogie
        bogies.clear();                //clear vector
        mtx.unlock();                  // unlock thread
        process_ready_ = false;        // set value to false in order to exit thread
    }
}
