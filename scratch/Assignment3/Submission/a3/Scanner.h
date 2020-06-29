#ifndef SCANNER_H
#define SCANNER_H
/**
 * @file Scanner.h
 * @author Esteban Andrade
 * @brief Will inherit from ScannerInterface.h
 * @version 0.1
 * @date 2020-05-26
 * @see ScannerInterface.h
 * @copyright Copyright (c) 2020
 * 
 */
#include "simulator.h"
#include "types.h"
#include <cmath>
#include <vector>
#include "ScannerInterface.h"

/**
 * @brief The class Scanner will perform all the analysis  relative to the aircraft and base in respect to the bogies
 * 
 */
class Scanner : public ScannerInterface
{
    /**
     * @brief Class variables
     * @note Metres refer to distance, Theta refers to angle in radians
     *
     * @privatesection
     */
private:
    std::vector<std::pair<double, double>> base_scanner_;              //!< vector of pairs of double that will obtain data from the base in (metres,timestamp)
    std::vector<std::pair<double, double>> friendly_scanner_;          //!< vector of pairs that will hold the data from the friendly in (metres, theta)
    std::vector<std::pair<double, double>> friendly_relative_to_base_; //!< vector of pairs that holds data  from the friendly relative to the base in (metres , theta)
    std::vector<RangeVelocityStamped> base_data_scan_;                 //!< vector that will hold data obtained from the base
    std::vector<RangeBearingStamped> friendly_data_scan_;              //!< vector that will hold data from the friendly
    std::vector<std::pair<double, double>> target_;                    //!< vector of pairs that will hold the target bogie position in x and y
    int bogie_index_;                                                  //!< number of bogie target
    std::vector<std::pair<double, double>> predicted_target_;          //!< vector of pairs that will hold the predicted bogie position in x and y
    std::vector<double> velocity_object_;                              //!< vector of double that will hold the Scalar of the velocity Vector of  the bogie
    double bearing_angle_;                                             //!< Bogie bearing angle relative to friendly
    double angle_;                                                     //!< will be the analysis product of the relationship between the bearing angle and the friendly orientation
    double predicted_orientation_;                                     //!< Predicted orientation of the bogie in space
    std::vector<std::pair<double, double>> velocity_vector_;           //!< Vector of pairs of double that will hold the velocity vector (Vx and Vy)

    /**
 * @publicsection
 * 
 */
public:
    /**
 * @brief Construct a new Scanner object
 * 
 */
    Scanner();
    /**
     * @brief Destroy the Scanner object
     * 
     */
    ~Scanner();
    /**
     * @brief Will determine the target bogie. it will select the most appropiate bogie and store its position in x and y
     * @param[in] Pose of the corresponding bogie object 
     * @param[in] Vector of RangeBearingStamped in order to analyse the the angle and position
     */
    void determineTarget(Pose, std::vector<RangeBearingStamped>);
    /**
     * @brief Will get and analise the distance from the base relative to the friendly. 
     * @note Will convert the coordinates to tangential distande
     * @param[in] Pose of the friendly 
     * 
     */
    void friendlyRelativeBase(Pose &);
    /**
     * @brief Will Scan and store the Values from the friendly scanner. 
     * @note it will Store the range of the scan
     * @param[in] Vector of RangeVelocityStamped 
     * 
     */
    void baseScan(std::vector<RangeVelocityStamped> &);
    /**
     * @brief Will analyse and the passed vector of RangeVelocityStamped and it will sort the results
     * @note Will store the in pairs of range and theta of the bogies relative to the friendly
     * @param[in] vector RangeBearingStamped 
     * 
     */
    void airCraftScan(std::vector<RangeBearingStamped> &);
    /**
     * @brief Get the Base Scan Results object from base scam
     * 
     * @return std::vector<RangeVelocityStamped> of BaseScan
     */
    std::vector<RangeVelocityStamped> getBaseScanResults();
    /**
     * @brief Get the Friendly Scan Results object from aircraft scan
     * 
     * @return std::vector<RangeBearingStamped> FriendlyScan
     */
    std::vector<RangeBearingStamped> getFriendlyScanResults();
    /**
     * @brief Get the Target object of the bogie
     * @note Will get the coordinates in X and Y in the plane 
     * @return std::vector<std::pair<double, double>> Coordinates 
     */
    std::vector<std::pair<double, double>> getTarget();
    /**
     * @brief Will reset all the data and STL containers
     * 
     */
    void resetData();
    /**
     * @brief Get the Base Scan object from the base Scan
     * @note will return range and time stamp
     * @return std::vector<std::pair<double, double>>  BaseScanCoordinates
     */
    std::vector<std::pair<double, double>> getBaseScan();
    /**
     * @brief Get the Friendly Scan object from the Aircraft scan 
     * @note Will get the values in Range and Bearing
     * @return std::vector<std::pair<double, double>> FriendlyScanCoordinates
     */
    std::vector<std::pair<double, double>> getFriendlyScan();
    /**
     * @brief Get the Base To Friendly object relative to the base
     * @note Will get values of distance and angle
     * @return std::vector<std::pair<double, double>> BaseToFriendlyCoordinates
     */
    std::vector<std::pair<double, double>> getBaseToFriendly();
    /**
     * @brief Get the Bogie Index object
     * 
     * @return int of bogie index target
     */
    int getBogieIndex();
    /**
     * @brief will Analyse the predicted position and compute the velocity vector
     * @note Will obtain the previous calculated target and get the position in two different times. To later calculate the velocity vector and predict the fiture position in 210 ms
     * @note Will consider all possible orientations and directions and adjust the predicted orientation
     * @param[in] std::vector<RangeBearingStamped> in order to get the intitial time stamp of the required objet be analysed
     * @param[in] std::vector<RangeBearingStamped> in order to get the second time stamp of the required object
     * 
     */
    void predictTarget(std::vector<RangeBearingStamped> &, std::vector<RangeBearingStamped> &);
    /**
    * @brief Get the Object Velocity object in Scalar
    * 
    * @return std::vector<double> of the scalar velocity
    */
    std::vector<double> getObjectVelocity();
    /**
     * @brief Get the Predicted Target object in position x and y 
     * @note will get the predicted coordinates in x and y 
     * @return std::vector<std::pair<double, double>> predicted_coordinates
     */
    std::vector<std::pair<double, double>> getPredictedTarget();
    /**
     * @brief Get the Bearing Angle object relative to the friendly and bogie
     * 
     * @return double of bearing angle
     */
    double getBearingAngle();
    /**
     * @brief Get the Velocity Vector object
     * @note Will get the velocity vector Vx and Vy
     * @return std::vector<std::pair<double, double>>  Velocity
     */
    std::vector<std::pair<double, double>> getVelocityVector();
    /**
     * @brief Get the Orientation Prediction object of the bogie
     * @note will get the predicted bogie orientation
     * @return double orientation
     */
    double getOrientationPrediction();
};

#endif //SCANNER_H