#include <iostream>
#include <thread>
#include "DataProcessing.h"

// std::condition_variable cv;

using namespace std;

// The function generates samples

int main()
{
  // std::shared_ptr<Class> pointer (new Class(nh)); [constructor is class (nh)]
  //std::thread t(&class::function,pointer);
  std::shared_ptr<DataProcessing> dataProccesingPointer(new DataProcessing());

  // Create the threads
  thread inc_thread(&DataProcessing::generateSamples, dataProccesingPointer);
  thread print_thread(&DataProcessing::processSamples, dataProccesingPointer);

  // Wait for the threads to finish (they wont)
  inc_thread.join();
  print_thread.join();

  return 0;
}
