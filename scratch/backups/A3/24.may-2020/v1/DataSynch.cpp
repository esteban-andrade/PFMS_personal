#include "DataSynch.h"
#include <vector>
#include <mutex>
#include <condition_variable>

DataSynch::DataSynch()
{
}

DataSynch::~DataSynch()
{
}
void DataSynch::dataThread(Navigation &navigate, const std::shared_ptr<Simulator> &sim, Pose friendly)
{
    while (true)
    {
        Pose friendly_simulator = sim->getFriendlyPose();
        navigate.navigate(friendly_simulator, friendly);
        navigate.purePursuit(sim, closest_bogie_, friendly);
        double lin_vel = navigate.getLinearVelocity();
        double ang_vel = navigate.getAngularSpeed();
        sim->controlFriendly(lin_vel, ang_vel);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

void DataSynch::controlThread(const std::shared_ptr<Simulator> &sim, Navigation &navigate, Scanner &scan, DataSynch &data)
{
    int i = 0;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::unique_lock<std::mutex> locker(mtx);
        std::vector<Pose> poses;
        scan.baseScan(sim);
        scan.airCraftScan(sim);
        std::cout << "[" << sim->elapsed() / 1000 << "s]" << std::endl;
        std::cout << "Friendly {x, y, orientation}:" << std::endl;
        std::cout << "  - x: " << navigate.getCurrentPose().position.x << "m" << std::endl;
        std::cout << "  - y: " << navigate.getCurrentPose().position.y << "m" << std::endl;
        std::cout << "  - orient: " << navigate.getCurrentPose().orientation * 180 / M_PI << " deg" << std::endl
                  << std::endl;

        scan.determineTarget(sim);
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        scan.friendlyRelariveBase(sim);
        scan.predictTarget(sim);
        Pose pose;
        std::cout << "x:" << scan.getTarget().at(i).first << " y " << scan.getTarget().at(i).second << std::endl;
        pose.position.x = scan.getPredictedTarget().at(i).first;
        pose.position.y = scan.getPredictedTarget().at(i).second;
        //pose.orientation = scan.getBearingAngle() + navigate.getOrientation(sim, pose);
        //pose.orientation = atan2(pose.position.y, pose.position.x);
        pose.orientation = scan.getOrientationPrediction();
        poses.push_back(pose);
        sim->testPose(poses);

        //std::cout << " Distance from base " << scan.getBaseScan().at(i).first << std::endl;
        std::cout << " Distance from friendly " << scan.getFriendlyScan().at(0).first << std::endl;
        std::cout << " Distance friendly from base " << scan.getBaseToFriendly().at(0).first << std::endl;

        std::cout << " Predicted x:" << scan.getPredictedTarget().at(0).first << " Predicted y " << scan.getPredictedTarget().at(0).second << std::endl;
        std::cout << " Bearing Angle :" << scan.getBearingAngle() << std::endl;
        // std::cout << " Bogie orientation " << scan.getBaseToFriendly().at(0).second << "\tBogie range " << scan.getBaseToFriendly().at(i).first << std::endl;
        std::cout << " Vx " << scan.getVelocityVector().at(0).first << "\tVy " << scan.getBaseToFriendly().at(0).second << " Velocity Scalar " << scan.getObjectVelocity().at(0) << std::endl;
        i++;
        if (i > 3)
        {
            i = 0;
            scan.resetData();
            //poses.clear();
        }
        process_ready_ = true;
        mtx.unlock();
        cv.notify_all();
    }
}

void DataSynch::processThread(Scanner &scanner)
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::unique_lock<std::mutex> locker(mtx);
        cv.wait(locker, [&] { return (process_ready_); });
        std::vector<RangeBearingStamped> bogies = scanner.getFriendlyScanResults();

        sort(bogies.begin(), bogies.end(), [](const RangeBearingStamped &lhs, const RangeBearingStamped &rhs) {
            return lhs.range < rhs.range;
        });

        closest_bogie_ = bogies.at(0);

        bogies.clear();
        mtx.unlock();
        process_ready_ = false;
    }
}
