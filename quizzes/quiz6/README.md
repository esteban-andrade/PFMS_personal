# Quiz 5

## Part A

You have been provided a library that implement a Radar sensor. The header file describes the public functions available and settings [radar.h](./a/dep/radar.h)

1. TASK: Create an instance of radar, obtain and display the default max Distance.

2. TASK: Create a thread tied to the spawn member function of Radar [radar.h](./a/dep/radar.h). Create a while loop in the main runs 50 times and displays the return value of getData member funcion.

3. TASK: We were not provided a rate for the getData, using the chrono library and 50 sucsessive calls to getData in the while loop of your main you have already developed, compute the refresh rate (running rate) of getData (This will tell us the scanning time of the sensor).

4. TASK: The scanning time is dependent on the MaxDistance, add to your main code that sets the other supported MaxDistance and another while loop that queries getData another 50 times. Using the chrono library and these 50 sucsessive calls to getData in the while loop, compute the refresh rate (running rate) of getData (This will tell us the scanning time of the sensor in the other supported configuration).

5. QUESTION: We have been provided the Radar as a library, did we need to implement functions defined in radar.h (YES/NO)?
   NO. These functions were already implemented in the library. we can only use this to access data from the main. Hence we cant change this.

## Part B

1. TASK: Look at the code in [folder b](./b), and you will notice that a [library](./b/complex.h) for performing operations on complex numbers has been written. Compile and run the test [executable](./b/test_complex.cpp). The unit test `Divide` will fail. Fix the static method `divide()` in [complex.cpp](./b/complex.cpp) and verify that the test passes after re-compilation.

2. TASK: Write separate unit tests in [test_complex.cpp](./b/test_complex.cpp) for each function within [complex.cpp](./b/complex.cpp).

3. TASK: Make sure that you can compile and run the executable. Your tests should pick up a bug in the [complex library](./b/complex.cpp). Fix the bug and make sure all tests pass.

4. QUESTION: What are some of the benefits of unit testing?

- Make the process agile:Unit testing really goes hand in hand with agile programming of all flavors because it builds in tests that allow you to make changes more easily. In other words, unit tests facilitate safe refactoring
  -Quality of code: Unit testing improves the quality of the code; it identifies every defect which may have aroused, before code is sent further for integration testing
  -FIND SOFTWARE BUGS EARLY:Issues are found at early stage. Since unit testing are carried out by developers where they test their individual code before the integration, issues can be found very early and can be resolved then and there without impacting the other piece of codes. This includes both bugs in the programmer’s implementation and flaws or missing parts of the specification for the unit.
  -FACILITATES CHANGES & SIMPLIFIES INTEGRATION:Unit testing allows the programmer to refactor code or upgrade system libraries at a later date, and make sure the module still works correctly. Unit tests detect changes which may break a design contract. They help with maintaining and changing the code.
  -PROVIDES DOCUMENTATION:Unit testing provides a documentation of the system
  -DEBUGGING PROCESS:Unit testing helps in simplifying the debugging process. If a test fails then only latest changes made in code needs to be debugged.
  -DESIGN:Writing the test first forces you to think through your design and what it must accomplish before you write the code
  ref: https://apiumhub.com/tech-blog-barcelona/top-benefits-of-unit-testing/

5. QUESTION: Why would a single unit test, testing all the functions in the complex library, be generally considered bad practice?
   This is because unit test works with a specific functionality, and it doesnt conver it completely. And it is bad practise to create something called testAllThings and call every method in the namespace.
   Another important aspect of a unit test is that it works with the use case in isolation. Which means it should not depend or be bothered by any external entity. So changing another part of the codebase should not affect a unit test case.
   Also the main purpose of uni test is to testing small parts of the code in isolation and a testAllThings will go against this framework purpose.
   Also, this entire TestAllthings wont be accurate as the final output maybe have an error, and it would be very difficult to find the error of an entire program. Thus is better to to test specific sections of the code to rather than all a sungle unit test of all the entire libraries.
