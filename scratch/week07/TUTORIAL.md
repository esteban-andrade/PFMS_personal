Week 7 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session.

Convars - Ex01
--------------------

Consider the [exo1 starter code](./starter/exo1) example which uses two threads to do some work in parallel, questions to reflect on:
* What are the data structures used between threads, what role do the threads have? 
* Could we use a lock_guard or unique_lock instead of managing the mutex locks 
* What are the roles of the sleep_for ? 
* What is CPU usage of this process?
* What value would you expect to see printed? 
* If you remove the cout in process thread does that have an effect?
* Where would you keep data?

Compile and run the code

TASK:
* guarantee the process sample runs immediately after new data is generated?
* Implement in `processSample` thread capacity to remove the sample closest to the mean?

 REFLECTION:
* In this contrived example of producer / consumer did we need the two threads? What if we did a significant amount of processing before with the data?

Convars - Ex02
--------------------

Embed the data, mutex, convar and functionality in one class, make the main launch the threads tied to functions of this class.

Threading - Ex03
--------------------

We have been provided source code of a sensor (radar), with a blocking call function [getData](./starter/ex02/radar.h), thus rendering the main blocked until data is available.  We are required to find the closest target from 3 radars

Complete the following:
* Change the Radar class, so the thread of execution is inside the sensor via the [start](./starter/ex02/radar.h) member function (the class will run now with a seperate thread generating data
* Create 3 instances of the radar class and run the corresponding threads

Now create the Data Processing class that would find the closest target
* What do we need to pass to this class

Implement the following
* Find the closest target from all 3 radars every 50ms, how could we guarantee this occurs
* Find the closest target from all 3 radars whenever a radar provides new data


