#include <thread> //For accessing the standard C++ thread library
#include <chrono> //Used for creating thread delays
#include <iostream>
#include <mutex>
#include <queue>
#include <condition_variable>

//std::condition_variable cv;
//std::mutex mtx;
//Define a new data structure called 'TDataBuffer' and instansiate a global variable
//of this structure called 'sequence'.
struct TDataBuffer
{
  std::mutex mtx;
  std::condition_variable cv;
  std::queue<long> buffer;
};

void fibonacci(TDataBuffer &sequence)
{
  static long a = 0;
  static long b = 1;
  // std::lock_guard<std::mutex> guard(sequence.mtx);
  sequence.buffer.push(a);

  while (true)
  {
    sequence.mtx.lock();
    //mtx.lock();

    sequence.buffer.push(b);
    b = a + b;
    a = b - a;
    sequence.mtx.unlock();

    //mtx.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(750));
    sequence.cv.notify_one();
  }
}

void printToTerminal(TDataBuffer &sequence)
{
  //std::lock_guard<std::mutex> lock(mtx);

  while (true)
  {
    //sequence.mtx.lock();
    //std::unique_lock<std::mutex> lock(mtx);
    std::unique_lock<std::mutex> lock(sequence.mtx);
    //mtx.lock();
    sequence.cv.wait(lock, [&] { return !sequence.buffer.empty(); });
    if (sequence.buffer.empty())
    {
      //sequence.mtx.lock();
      std::cout << "<buffer was empty>" << std::endl;
      //sequence.mtx.unlock();
    }
    else
    {
      long next = sequence.buffer.front();
      sequence.buffer.pop();
      std::cout << "Next in sequence: " << next << std::endl;
      // mtx.unlock();
      // mtx.lock();
    }
    //sequence.mtx.unlock();
    lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}

int main(void)
{
  TDataBuffer sequence;
  //Create the threads
  std::thread producer(fibonacci, std::ref(sequence));
  std::thread consumer(printToTerminal, std::ref(sequence));

  producer.join();
  consumer.join();

  return 0;
}
