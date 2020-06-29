#include <thread>
#include <vector>
#include "bogie.h"

Bogie::Bogie(){
}

void Bogie::setSimulator(const std::shared_ptr<Simulator> & sm){
    sim_ = sm;
}

std::vector<RangeVelocityStamped> Bogie::getBogiesFromBase(){
    return bogies_from_base_;
}

void Bogie::setBogiesFromBase(){
    bogies_from_base_ = sim_->rangeVelocityToBogiesFromBase();
}

std::vector<RangeBearingStamped> Bogie::getBogiesFromFriend(){
    return bogies_from_friend_;
}

void Bogie::setBogiesFromFriend(){
    bogies_from_friend_ = sim_->rangeBearingToBogiesFromFriendly();
}

Pose Bogie::getCurrentPose(){
    return current_pose_;
}

double Bogie::getLinSpeed(){
    return lin_velocity_;
}

double Bogie::getAngSpeed(){
    return ang_velocity_;    
}