#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

std::mutex mtx;

int shared_int = 0;

void increment(int &shared_int)
{
   mtx.lock();
   for (int i = 0; i < 10000000; ++i)
   {
      shared_int++;
      //tracker = i;
   }
   mtx.unlock();
   //std::cout << "From Thread ID:" << std::this_thread::get_id() << "\nFinal value: " << shared_int << std::endl;
}

int main()
{

   std::vector<std::thread> threads;
   std::thread th1(increment, std::ref(shared_int));
   std::thread th2(increment, std::ref(shared_int));
   threads.push_back(std::move(th1));
   threads.push_back(std::move(th2));

   for (std::thread &th : threads)
   {
      if (th.joinable())
         th.join();
   }

   // th1.join(); //waits for t1 to finish th1.detach() th1 will be freely on its own --daemon process
   // th2.join();
   std::cout << "Final value: " << shared_int << std::endl;
   return 0;
}
