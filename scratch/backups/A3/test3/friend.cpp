#include <thread>
#include <vector>
#include "friend.h"

#define AIRSPACE_SIZE 4000

Friendly::Friendly(){
    inside_airspace_ = true;
}

void Friendly::setSimulator(const std::shared_ptr<Simulator> & sm){
    sim_=sm;
}

Pose Friendly::getCurrentPose(){
    return current_pose_;
}

void Friendly::setCurrentPose(){
    current_pose_ = sim_->getFriendlyPose();
}

bool Friendly::inAirspace(){
    return inside_airspace_;
}

double Friendly::getLinSpeed(){
    return lin_velocity_;
}

void Friendly::setLinSpeed(){
    sim_->getFriendlyLinearVelocity();
}

double Friendly::getAngSpeed(){
    return ang_velocity_;
}

void Friendly::setAngSpeed(){
    sim_->getFriendlyAngularVelocity();
}

void Friendly::checkBoundary(){
    double currentOrientation = sim_->getFriendlyPose().orientation;
    double lowerLim, upperLim, angular_vel;

    if (currentOrientation > M_PI){
        lowerLim = currentOrientation-M_PI - M_PI*0.1;
        upperLim = currentOrientation-M_PI + M_PI*0.1;
    }
    else{
        lowerLim = currentOrientation+M_PI - M_PI*0.1;
        upperLim = currentOrientation+M_PI + M_PI*0.1;
    }

    if (sim_->getFriendlyPose().position.x >= AIRSPACE_SIZE - 200 || sim_->getFriendlyPose().position.x <= -AIRSPACE_SIZE - 200 || sim_->getFriendlyPose().position.y >= AIRSPACE_SIZE - 200 || sim_->getFriendlyPose().position.y <= -AIRSPACE_SIZE - 200){
        inside_airspace_ = false;

        if (sim_->getFriendlyPose().position.x >= AIRSPACE_SIZE - 200 && currentOrientation >= 1.5*M_PI ||
            sim_->getFriendlyPose().position.x <= -AIRSPACE_SIZE - 200 && currentOrientation <= M_PI ||
            sim_->getFriendlyPose().position.y >= AIRSPACE_SIZE - 200 && (currentOrientation <= 0.5*M_PI || currentOrientation >= 1.5*M_PI) ||
            sim_->getFriendlyPose().position.y <= -AIRSPACE_SIZE - 200 && (currentOrientation <= 1.5*M_PI || currentOrientation >= 1.9*M_PI)){
            angular_vel = -1.1772;
        }
        else if (sim_->getFriendlyPose().position.x >= AIRSPACE_SIZE - 200 && currentOrientation < 0.5*M_PI ||
                sim_->getFriendlyPose().position.x <= -AIRSPACE_SIZE - 200 && currentOrientation > M_PI ||
                sim_->getFriendlyPose().position.y >= AIRSPACE_SIZE - 200 && (currentOrientation > 0.5*M_PI || currentOrientation <= 1.5*M_PI) ||
                sim_->getFriendlyPose().position.y <= -AIRSPACE_SIZE - 200 && (currentOrientation > 1.5*M_PI || currentOrientation <= 0.5*M_PI)){
            angular_vel = 1.1772;
        }

        while (!(inside_airspace_)){

            if (sim_->getFriendlyPose().orientation > upperLim || sim_->getFriendlyPose().orientation < lowerLim){
                sim_->controlFriendly(50, angular_vel);
            }
            else{
                inside_airspace_ = true;
            }
        }
    }
}

void Friendly::destroy(const RangeBearingStamped & nearest_bogie){
    double lin, ang;

    // if (target_acquired){
    if (nearest_bogie.bearing < 2*M_PI-0.2 && nearest_bogie.bearing >= 2*M_PI-0.5){
        lin=500;
        ang= -0.11772;
    }
    else if (nearest_bogie.bearing < 0.5 && nearest_bogie.bearing >= 0.2){
        lin=500;
        ang= 0.11772;
    }
    else if (nearest_bogie.bearing < 2*M_PI-0.5 && nearest_bogie.bearing >= M_PI){
        lin = 50;
        ang = -1.1772;
    }
    else if (nearest_bogie.bearing >= 0.5 && nearest_bogie.bearing < M_PI){
        lin = 50;
        ang = 1.1772;
    }
    else if (nearest_bogie.bearing < 0.2 || nearest_bogie.bearing > 2*M_PI - 0.2 || nearest_bogie.range > 3000){
        lin = 900;
        ang = 0;
    }
    // }

    sim_->controlFriendly(lin, ang);
}

