#ifndef DATASYNCH_
#define DATASYNCH_
/**
 * @file DataSynch.h
 * @author Esteban Andrade
 * @brief This Class will allow to synchronize all the classes and allow them to work effectively
 * @version 0.1
 * @date 2020-05-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <vector>
#include <mutex>
#include <condition_variable>
#include "Navigation.h"
#include "Scanner.h"
#include "simulator.h"
#include "types.h"
/**
 * @brief The DataSynch will allow to synchronize all the data and objects. This allow to interconnect multiple classes and ensire that the data is protected and secured
 * 
 */
class DataSynch
{
    /**
     * @privatesection
     * 
     */
private:
    std::mutex mtx;                                          //!< mutex for data protection
    std::condition_variable cv;                              //!< conditional variable for data synchronization
    RangeBearingStamped closest_bogie_;                      //!< RangeBearingStamped
    std::vector<RangeBearingStamped> friendly_scan_results_; //!< Vector that will hold the friendly scan in real time
    bool process_ready_;                                     //!< Boolean to check if the process is ready for unlock
    std::vector<RangeVelocityStamped> base_scan_sim_;        //!< Vector that will hold data from the base scan in real time
    Pose pose_simulation_;                                   //!< Will hold the pose in real time
    Pose pose_;                                              //!< Will hold the predicted pose in position x,y,and orientation
    std::vector<Pose> poses_;                                //!< Will aid to render the predicted pose
    std::vector<RangeBearingStamped> scan_prediction_;       //!< Will aid to get second information for analysis
    Pose friendly_simulator_;                                //!< will hold the friendly pose in real time for navigation
    std::vector<RangeBearingStamped> bogies;                 //!< vector that will keep getting new targets once current is destroyed
    /**
 * @publicsection
 * 
 */
public:
    /**
 * @brief Construct a new Data Synch object that will be used for synchronization
 * 
 */
    DataSynch();
    /**
     * @brief Destroy the Data Synch object
     * 
     */
    ~DataSynch();
    /**
     * @brief The Data thread will aid to contain all the data that will be used for navigation
     * @note Will control the data of navigation and pure pursuit algorithm
     * @param[in] Navigation object that will control the orientation
     * @param[in] std::shared_ptr<Simulator> simulator point to keep getting dara
     * @param[in] Pose to get friendly pose
     */
    void dataThread(Navigation &, const std::shared_ptr<Simulator> &, Pose);
    /**
     * @brief The control thread will control all the data that is obtained from all the sensor in order to estimate the pose of the bogie in the future and wil notify that the data is ready
     * @note There is a debug here out
     * @note The pose will only be 4 intances of poses in the render and this will get refreshed over time
     * @param[in] std::shared_ptr<Simulator> in order to get data in realtime
     * @param[in] Navigation object that will allow to get information regarding  the current status of the aircraft
     * @param[in] Scanner object will allow to get data from all the sensor and estime the future pose
     * @param[in] DataSynch object will allow to synchronize all the data
     */
    void controlThread(const std::shared_ptr<Simulator> &, Navigation &, Scanner &, DataSynch &);
    /**
     * @brief The proecess thread will allow to keep track of the bogie and pick a new one everytime one is destroyed
     * @note once the process is completed the process will be repeated
     * @param[in] Scanner object that will aid to get the scan results in real time
     * 
     */
    void processThread(Scanner &);
};

#endif //DATASYNCH_