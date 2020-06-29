Quiz 5
======

Part A
------
You have been provided a library that implement a Radar sensor. The header file describes the public functions available and settings [radar.h](./a/dep/radar.h)

1) TASK: Create an instance of radar, obtain and display the default max Distance.

2) TASK: Create a thread tied to the spawn member function of Radar [radar.h](./a/dep/radar.h). Create a while loop in the main runs 50 times and displays the return value of getData member funcion.

3) TASK: We were not provided a rate for the getData, using the chrono library and 50 sucsessive calls to getData in the while loop of your main you have already developed, compute the refresh rate (running rate) of getData (This will tell us the scanning time of the sensor).

4) TASK: The scanning time is dependent on the MaxDistance, add to your main code that sets the other supported MaxDistance and another while loop that queries getData another 50 times. Using the chrono library and these 50 sucsessive calls to getData in the while loop, compute the refresh rate (running rate) of getData (This will tell us the scanning time of the sensor in the other supported configuration).

5) QUESTION: We have been provided the Radar as a library, did we need to implement functions defined in radar.h (YES/NO)?

Part B
------
1) TASK: Look at the code in [folder b](./b), and you will notice that a [library](./b/complex.h) for performing operations on complex numbers has been written. Compile and run the test [executable](./b/test_complex.cpp). The unit test `Divide` will fail. Fix the static method `divide()` in [complex.cpp](./b/complex.cpp) and verify that the test passes after re-compilation.

2) TASK: Write separate unit tests in [test_complex.cpp](./b/test_complex.cpp) for each function within [complex.cpp](./b/complex.cpp).

3) TASK: Make sure that you can compile and run the executable. Your tests should pick up a bug in the [complex library](./b/complex.cpp). Fix the bug and make sure all tests pass.

4) QUESTION: What are some of the benefits of unit testing?

5) QUESTION: Why would a single unit test, testing all the functions in the complex library, be generally considered bad practice?
