
#include <thread>
#include <iostream>
#include "DataProcessing.h"
using std::condition_variable;
using std::cout;
using std::endl;
using std::mutex;
using std::vector;

DataProcessing::DataProcessing()
{
}

DataProcessing::~DataProcessing()
{
}
void DataProcessing::generateSamples()
{

    //Setup and seed our random normal distribution generator
    std::default_random_engine generator(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    std::normal_distribution<double> distribution(6.0, 5.0); //mean of 6m and a stdev of 5m

    while (true)
    {

        // This delay is included to improve the emulate some other process of generating the data
        // by the sensor which could be at a specific rate
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        //numMutex.lock();
        std::unique_lock<std::mutex> lock(numMutex);

        cout << "sample gen" << endl;
        // We only access num while the mutex is locked
        double sample = distribution(generator);
        data.push_back(sample);
        cv.notify_one();

        //numMutex.unlock();
        lock.unlock();
    }
}

// This function consumes the samples
void DataProcessing::processSamples()
{
    //vector<double> &data, mutex &numMutex, condition_variable &cv
    while (true)
    {
        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        // numMutex.lock();

        // if (!data.empty()){
        //     double sample = data.back();
        //     data.pop_back();
        //     numMutex.unlock();
        //     // We now have a sample
        //     cout <<  "sample is:" << sample << endl;
        //   }
        //   else{
        //     numMutex.unlock();
        //     // This delay is to avoid hard-looping and consuming too much cpu
        //     // What is a better solution
        //     std::this_thread::sleep_for (std::chrono::milliseconds(20));
        //   }
        std::unique_lock<std::mutex> lock(numMutex);
        cv.wait(lock, [&] { return !data.empty(); });
        double sample = data.back();
        data.pop_back();
        // We now have a sample
        cout << "sample is:" << sample << endl;
        lock.unlock();
    }
}