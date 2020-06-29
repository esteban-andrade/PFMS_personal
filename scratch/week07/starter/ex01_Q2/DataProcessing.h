#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H

#include <mutex>
#include <vector>
#include <chrono>
#include <random>    // random number generation
#include <algorithm> // algorithms for sorting
#include <condition_variable>

class DataProcessing
{
private:
    std::vector<double> data;
    // We will use this mutex to synchonise access to num
    std::mutex numMutex;
    std::condition_variable cv;

public:
    DataProcessing();
    ~DataProcessing();
    void generateSamples();
    void processSamples();
};

#endif //DATAPROCESSING_H