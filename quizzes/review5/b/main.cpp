#include <thread> //For accessing the standard C++ thread library
#include <chrono> //Used for creating thread delays
#include <iostream>
#include <queue>
#include <mutex>

//Define a new data structure called 'TDataBuffer' and instansiate a global variable
//of this structure called 'sequence'.
struct TDataBuffer {
  std::queue<long> buffer;
  std::mutex mtx;
} sequence;

void fibonacci(void) {
  static long a = 0;
  static long b = 1;

  sequence.mtx.lock();
  sequence.buffer.push(a);
  sequence.mtx.unlock();
  while(true) {
       sequence.mtx.lock();
    sequence.buffer.push(b);
     sequence.mtx.unlock();
    b = a + b;
    a = b - a;

    std::this_thread::sleep_for(std::chrono::milliseconds(750));
  }
}

void printToTerminal(void) {
  while(true) {
       sequence.mtx.lock();
    if(sequence.buffer.empty()) {
         sequence.mtx.unlock();
      std::cout << "<buffer was empty>" << std::endl;
    } else {
      long next = sequence.buffer.front();
      sequence.buffer.pop();
       sequence.mtx.unlock();

      std::cout << "Next in sequence: " << next << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}


int main (void) {
  //Create the threads
  std::thread producer(fibonacci);
  std::thread consumer(printToTerminal);

  producer.join();
  consumer.join();

  return 0;
}
