Quiz 0
======

Part A
------
1) Attempt to compile the code in [folder a](./a). Why does the code fail to compile?
The function printArray is declared twice. Accessing x through a pointer or an array is the same and gives the same function signature.
2) Fix the compilation error. Why does a segmentation fault (segfault) occur?
A segmentation fault occurs because arraySize is initialized as 10 so when the populate number function is called after the user inputs a character, a segmentation fault occurs because it tries to add to beyond the maximum size of the array

4) Create a function that prints only the elements of the array that are larger than : mean + one standard deviation

5) Create a function that assigns elements of array x to a vector named `vec` (HINT: decide on the correct type)

Part B
------
1) Look at the code in [folder b](./b). Implement the methods of the Sample class in the [sample.cpp](./b/sample.cpp) file based on the definition provided in [sample.h](./b/sample.h)

2) Make an executable that creates an object `sample` of `Sample` class and then obtains the value of parameter `value_` in this object.

3) What do we call functions of a class?
Function members or Methods
4) What access specifiers are used?
Public and private
5) What do we call variable `value_` in the `Sample` class?
Data member or property
