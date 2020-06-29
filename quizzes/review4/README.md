Quiz 4
======

Part A
------

1) TASK: Create a function that accepts a deque and modifies it by adding user specified numbers of elements, each element is form a gaussian distribution (mean: 8, std dev 4) [deque_vector_ops.cpp](./a/deque_vector_ops.h)

-Done

2) TASK: Create a functions that prints the chosen container

-Done

3) We are now tasked to design a function to rearrange elements of out container (vector or deque) by bubble sort operation,refer for pseudo code  (https://en.wikipedia.org/wiki/Bubble_sort).
QUESTION: Which STL container is more suitable for the Bubble sort and why? deque because it allows efficient insertion at the beginning and end of the structure

4) TASK: Create a function that accept the chosen container and rearranges elements by bubble sort operation

-Done

5) TASK: In the main call sorting function and print the container after operation (re-use function developed in step 2)

-Done

Part C
-------

Consider the code in [data_race.cpp](./b/data_race.cpp)  

1) QUESTION: How many threads are running in parallel (disregarding the main)? 3


2) QUESTION: What value would you expect to see printed? 10000000


3) QUESTION: What is the specific problem with this code, causing it to fail? two threads are trying to adquire the value

4) TASK: Implement one approach to fix the problem and outline merits of the solution.

5) TASK: Instead of having `int shared_int` as a global variable, change the code to pass this variable to function `increment ()`


