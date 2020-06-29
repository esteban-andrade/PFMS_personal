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
        navigate.navigate(sim, friendly);

        navigate.purePursuit(sim, closest_bogie_);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void DataSynch::controlThread(const std::shared_ptr<Simulator> &sim, Navigation &navigate, Scanner &scan, DataSynch &data)
{
    int i = 0;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        scan.friendlyRelariveBase(sim);
        scan.predictTarget(sim);
        Pose pose;
        std::cout << "x:" << scan.getTarget().at(i).first << " y " << scan.getTarget().at(i).second << std::endl;
        pose.position.x = scan.getPredictedTarget().at(i).first;
        pose.position.y = scan.getPredictedTarget().at(i).second;
        //pose.orientation = scan.getBearingAngle() + navigate.getOrientation(sim, pose);
        pose.orientation = tan(pose.position.y / pose.position.x);
        poses.push_back(pose);
        sim->testPose(poses);

        //std::cout << " Distance from base " << scan.getBaseScan().at(i).first << std::endl;
        std::cout << " Distance from friendly " << scan.getFriendlyScan().at(0).first << std::endl;
        std::cout << " Distance friendly from base " << scan.getBaseToFriendly().at(0).first << std::endl;

        std::cout << " Predicted x:" << scan.getPredictedTarget().at(0).first << " Predicted y " << scan.getPredictedTarget().at(0).second << std::endl;
        std::cout << " Bearing Angle :" << scan.getBearingAngle() << std::endl;
        // std::cout << " Bogie orientation " << scan.getBaseToFriendly().at(0).second << "\tBogie range " << scan.getBaseToFriendly().at(i).first << std::endl;
        std::cout << " Vx " << scan.getVelocityVector().at(0).first << "\tVy " << scan.getBaseToFriendly().at(0).second << std::endl;
        i++;
        if (i > 3)
        {
            i = 0;
            scan.resetData();
            poses.clear();
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
// void DataSynch::setNearestBogieRange(Scanner &scanner)
// {
//     std::vector<RangeBearingStamped> bogies = scanner.getFriendlyScanResults();

//     sort(bogies.begin(), bogies.end(), [](const RangeBearingStamped &lhs, const RangeBearingStamped &rhs) {
//         return lhs.range < rhs.range;
//     });

//     closest_bogie_ = bogies.at(0);
//     bogies.clear();
// }
// GlobalOrd DataSynch::calculateCurrentPose(Navigation &navigation)
// {
//     double friendly_x = navigation.getCurrentPose().position.x;
//     double friendly_y = navigation.getCurrentPose().position.y;
//     double bogie_range = closest_bogie_.range;
//     bearing_target_ = closest_bogie_.bearing;
//     double target_x, target_y;
//     GlobalOrd bogie_;
//     if (bearing_target_ <= M_PI_2 && bearing_target_ >= 0)
//     {
//         target_x = bogie_range * cos(bearing_target_);
//         target_y = bogie_range * sin(bearing_target_);
//     }
//     else if (bearing_target_ > M_PI_2 && bearing_target_ <= M_PI)
//     {
//         target_x = -bogie_range * cos(M_PI - bearing_target_);
//         target_y = bogie_range * sin(M_PI - bearing_target_);
//     }
//     else if (bearing_target_ > M_PI && bearing_target_ <= M_PI * 1.5)
//     {
//         target_x = -bogie_range * cos(bearing_target_ - M_PI);
//         target_y = -bogie_range * sin(bearing_target_ - M_PI);
//     }
//     else if (bearing_target_ > M_PI * 1.5 && bearing_target_ <= M_PI * 2)
//     {
//         target_x = bogie_range * cos(2 * M_PI - bearing_target_);
//         target_y = -bogie_range * sin(2 * M_PI - bearing_target_);
//     }

//     bogie_.x = friendly_x + target_x;
//     bogie_.y = friendly_y + target_y;
//     return bogie_;
// }
// void DataSynch::calculateVelocity(Navigation &navigation)
// {
//     GlobalOrd p1, p2;
//     double p1_x, p1_y, p2_x, p2_y;
//     p1 = calculateCurrentPose(navigation);
//     p1_x = p1.x;
//     p1_y = p1.y;
//     std::this_thread::sleep_for(std::chrono::milliseconds(50));
//     p2 = calculateCurrentPose(navigation);
//     p2_x = p2.x;
//     p2_y = p2.y;
// }
// void DataSynch::predictPosition(Navigation &navigation)
// {

//     double current_x, current_y, future_x, future_y;

//     current_x = calculateCurrentPose(navigation).x;
//     current_y = calculateCurrentPose(navigation).y;
// }