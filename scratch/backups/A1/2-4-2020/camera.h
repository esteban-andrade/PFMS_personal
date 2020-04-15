#ifndef MONOCHROME_CAMERA_H
#define MON0CHROME_CAMERA_H

#include <string>
#include <vector>

// Declaration of the rectangle class
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
    int getSampleNumber (void);
    int getElapsedTime(void);
    int getSamplingTime(void);
    int getRowNumber (void);
    int getColumnNumer(void);
    void getData(void);
    void setSamplingTime(int sampling_time);
    void setImageSize(int rows, int columns);
    void setSampleNumber (int sample_number);
    bool queryParameters(void);
    // const std::vector<std::vector<int>> &GetVect() const;
    std::vector<std::vector<int>> printData();



    // Private members are only accessible from within methods of the same class
private:
    int sampling_time_;
    int elapsed_time_;
    int rows_;
    int columns_;
    int sample_number_ ;
    std::string model_;
    std::vector<std::vector<int>> image_size_;
};

#endif // MON0CHROME_CAMERA_H
