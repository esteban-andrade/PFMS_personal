#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <queue>

std::queue<int> queue;

void incrementNum(int &num, std::mutex &numMutex)
{
    while (true)
    {
        //Use mutex
        std::lock_guard<std::mutex> lock(numMutex);
        num++;
    }
}

void printNum(int &num, std::mutex &numMutex)
{
    while (true)
    {
        std::lock_guard<std::mutex> lock(numMutex);
        std::cout << num << std::endl;
    }
}
void sumNumberQueue(std::queue<int> &numbers, int &sum, std::mutex &mtx)
{
    // mtx.lock();
    //std::unique_lock<std::mutex> lock(mtx);
    std::lock_guard<std::mutex> lock(mtx);
    while (!numbers.empty())
    {
        // std::lock_guard<std::mutex> lock(mtx);

        sum += numbers.front();
        numbers.pop();
        mtx.unlock();
        mtx.lock();
    }
    //std::lock_guard<std::mutex> lock(mtx);
    //mtx.unlock();
}

int main()
{

    // int num = 0;

    // for (int i = -500000; i <= 500000; i++)
    // {
    //     queue.push(i);
    // }
    // // We will use this mutex to synchonise access to num
    // std::mutex numMutex;

    // // Create the threads, how do we create the thread? What are the parameters
    // std::thread inc_thread(incrementNum, std::ref(num), std::ref(numMutex));
    // std::thread print_thread(printNum, std::ref(num), std::ref(numMutex));

    // // Wait for the threads to finish (they wont)
    // inc_thread.join();
    // print_thread.join();

    std::queue<int> data;
    std::mutex mtx;
    int sum1 = 0, sum2 = 0;
    for (int i = 1; i <= 500000; i++)
    {
        data.push(i);
        data.push(-i);
    }

    std::thread th1(sumNumberQueue, std::ref(data), std::ref(sum1), std::ref(mtx));
    std::thread th2(sumNumberQueue, std::ref(data), std::ref(sum2), std::ref(mtx));

    th1.join();
    th2.join();

    std::cout << "SUM 1 = " << sum1 << "\nSUM 2 = " << sum2 << "\nTotal :" << sum1 + sum2 << std::endl;

    return 0;
}
