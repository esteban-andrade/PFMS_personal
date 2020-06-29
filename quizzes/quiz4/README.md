Quiz 4
======

Part A
------

1) TASK: Create a function that accepts a deque and modifies it by adding user specified numbers of elements, each element is form a gaussian distribution (mean: 8, std dev 4) [deque_vector_ops.cpp](./a/deque_vector_ops.h)

2) TASK: Create a functions that prints the chosen container

3) We are now tasked to design a function to rearrange elements of out container (vector or deque) by bubble sort operation,refer for pseudo code  (https://en.wikipedia.org/wiki/Bubble_sort).
QUESTION: Which STL container is more suitable for the Bubble sort and why?
In this case both will be suitable and the difference is that vectors can only add or delete elemets from the end and middle whereas a deque will perform start,middle and end. Thus in case of the bubble sort where elements will be sorted from the beginning the implementation of a DEQUE will be  more effective as it has better performance with elements that are at the beginning rather than at the end.  HOWEVER both will  be able to perform both processes. Refer to deque_vector_ops.cpp

4) TASK: Create a function that accept the chosen container and rearranges elements by bubble sort operation

5) TASK: In the main call sorting function and print the container after operation (re-use function developed in step 2)

Part C
-------

Consider the code in [data_race.cpp](./b/data_race.cpp)  

1) QUESTION: How many threads are running in parallel (disregarding the main)?
2 threads th1 and th2

2) QUESTION: What value would you expect to see printed?
The value will be 20000000 for both threads

3) QUESTION: What is the specific problem with this code, causing it to fail?
The data is exceeding the limits of the ranger and fluctuating the result.The final result is also not reaching the desired value of  20000000. Which shows that both threads are in a race condition (both threads fighting for the same resources) .Thus this suggest that the threads are not secure no thread protection. Mutex should be implemented to fix this.

4) TASK: Implement one approach to fix the problem and outline merits of the solution.


5) TASK: Instead of having `int shared_int` as a global variable, change the code to pass this variable to function `increment ()`


