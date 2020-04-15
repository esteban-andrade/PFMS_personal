/*
    Created by Esteban Andrade 12824583
    Sensor 3: Monochrome Camera
*/

#ifndef MONOCHROME_CAMERA_H
#define MON0CHROME_CAMERA_H

#include <string>
#include <vector>
#include <chrono>

// Declaration of the Camera class
class Camera
{
    // Public members are accessible from outside the class (ie. in main)
public:
    // Declare default the constructor
    Camera();
    // Declare constructor that sets values
    Camera(int &rows, int &columns, int &sampling_time);
    // Destructor
    ~Camera();

    //Methods
    std::string getModel(void);
    std::string getImageSize(void);
    int getSampleNumber(void);
    double getElapsedTime(void);
    int getSamplingTime(void);
    int getRowNumber(void);
    int getColumnNumer(void);
    void timeTracking(void);
    // will generete new data
    void pollData(void);
    void setSamplingTime(int sampling_time);
    void setImageSize(int rows, int columns);
    void setSampleNumber(int sample_number);
    // will verify parameters
    bool queryParameters(void);
    // will get the result data
    std::vector<std::vector<int>> getData();
    std::chrono::duration<double> getTimeFirstQuery();

private:
    int sampling_time_;
    // will get the elapsed time
    double elapsed_time_; 
    int config_change_time_;
    int time;
    int rows_;
    int columns_;
    int sample_number_;
    std::string model_;
    std::vector<std::vector<int>> image_size_;
    //will track the time sice there is a config change
    std::chrono::high_resolution_clock::time_point start_config_time_; 
    // will track the timer since initialization
    std::chrono::steady_clock::time_point timer_tracker_;
};

#endif // MON0CHROME_CAMERA_H
