#include <thread>
#include <vector>
#include <list>
#include "intercept_bogie.h"

InterceptBogies::InterceptBogies(const std::shared_ptr<Simulator> &sm)
{
    sim_ = sm;
    f1_.setSimulator(sm);
    b1_.setSimulator(sm);
    // target_acquired_ = false;
}

void InterceptBogies::controlThread()
{
    while (true)
    {
        //Feed the watchdog control timer
        double lin;
        double ang;

        f1_.checkBoundary();
        f1_.destroy(nearest_bogie_);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void InterceptBogies::dataThread()
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::unique_lock<std::mutex> lck(mu);
        //Get the friendly aircraft's position and orientation
        f1_.setCurrentPose();

        //placeholder
        f1_.setCurrentPose();
        f1_.setLinSpeed();
        f1_.setAngSpeed();

        std::vector<Pose> poses;
        poses.push_back(f1_.getCurrentPose());
        sim_->testPose(poses);

        b1_.setBogiesFromFriend();
        b1_.setBogiesFromBase();

        std::cout << "[" << sim_->elapsed() / 1000 << "s]" << std::endl;
        std::cout << "Friendly {x, y, orientation}:" << std::endl;
        std::cout << "  - x: " << f1_.getCurrentPose().position.x << "m" << std::endl;
        std::cout << "  - y: " << f1_.getCurrentPose().position.y << "m" << std::endl;
        std::cout << "  - orient: " << f1_.getCurrentPose().orientation * 180 / M_PI << " deg" << std::endl
                  << std::endl;
        process_ready_ = true;
        mu.unlock();
        cv.notify_all();
    }
}

void InterceptBogies::processThread()
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        std::unique_lock<std::mutex> lck(mu);
        cv.wait(lck, [&] { return (process_ready_); });

        this->setNearestBogieRange();

        mu.unlock();
        process_ready_ = false;
    }
}

void InterceptBogies::setNearestBogieRange()
{
    std::vector<RangeBearingStamped> bogies = b1_.getBogiesFromFriend();

    sort(bogies.begin(), bogies.end(), [](const RangeBearingStamped &lhs, const RangeBearingStamped &rhs) {
        return lhs.range < rhs.range;
    });

    nearest_bogie_ = bogies[0];
    bogies.clear();
}

GlobalOrd InterceptBogies::calculateCurrentPose()
{
    double friendly_x = f1_.getCurrentPose().position.x;
    double friendly_y = f1_.getCurrentPose().position.y;
    double bogie_range = nearest_bogie_.range;
    bogie_bearing_ = nearest_bogie_.bearing;
    // target_acquired_ = true;
    double target_x, target_y;
    GlobalOrd bogie_;

    if (bogie_bearing_ <= M_PI_2 && bogie_bearing_ >= 0)
    {
        target_x = bogie_range * cos(bogie_bearing_);
        target_y = bogie_range * sin(bogie_bearing_);
    }
    else if (bogie_bearing_ > M_PI_2 && bogie_bearing_ <= M_PI)
    {
        target_x = -bogie_range * cos(M_PI - bogie_bearing_);
        target_y = bogie_range * sin(M_PI - bogie_bearing_);
    }
    else if (bogie_bearing_ > M_PI && bogie_bearing_ <= M_PI * 1.5)
    {
        target_x = -bogie_range * cos(bogie_bearing_ - M_PI);
        target_y = -bogie_range * sin(bogie_bearing_ - M_PI);
    }
    else if (bogie_bearing_ > M_PI * 1.5 && bogie_bearing_ <= M_PI * 2)
    {
        target_x = bogie_range * cos(2 * M_PI - bogie_bearing_);
        target_y = -bogie_range * sin(2 * M_PI - bogie_bearing_);
    }

    bogie_.x = friendly_x + target_x;
    bogie_.y = friendly_y + target_y;
    return bogie_;
}

void InterceptBogies::calculateVelocity()
{
    GlobalOrd p1, p2;
    double p1_x, p1_y, p2_x, p2_y;
    p1 = this->calculateCurrentPose();
    p1_x = p1.x;
    p1_y = p1.y;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    p2 = this->calculateCurrentPose();
    p2_x = p2.x;
    p2_y = p2.y;
    //velocity_x_ = sim_->distance(p1_x, p1_y)/50;
    //velocity_x_ = sim_->distance(p2_x, p2_y)/50;
}

void InterceptBogies::predictPos()
{
    double current_x, current_y, future_x, future_y;

    current_x = this->calculateCurrentPose().x;
    current_y = this->calculateCurrentPose().y;
}