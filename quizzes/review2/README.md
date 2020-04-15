Quiz 2
======

Part A
------

1) TASK: Modify the base class `Shape` file [shape.h](./a/shape.h) such that function getArea() is defined in `Shape`, and the child classes are required to implement this function.  
HINT: Polymorphism and the concept of virtual.

 - Made get area a virtual function in shape

2) TASK: Create a Square and Traingle, and store both of them in a vector of `Shape` pointers

 - Done

3) TASK: Add `Cicrle` as a child class of `Shape`.
HINT: Use the Circle class developed in Week 03

 - Done

4) TASK: Create a function that loops through shapes and display their area.
HINT: Think of the function signature.

 - Done

5) TASK: Write a program that allows the user to specify number of circles and `max_radius`. Create the shapes with random lengths to be capped to `max_length`.


Part B
------

1) TASK: Modify the file [car.h](./a/car.h) so that it inherits behaviour from the base class, [controllerinterface](./a/controllerinterface.h).

 - Added public Controllerinterface to car.h

2) TASK: Instatiate two objects of type `Car` with different specifications and determine's their top speed.

 - Done

3) QUESTION: When instantiating an object of the car class, which constructor(s) are caled and in what order.

 - COntrollerInterface then Car

4) TASK: Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and determines the time difference between the two vehicles reaching zero.

 - Done

5) QUESTION: Class [controllerinterface](./a/controllerinterface.h) is a special class,  and has special member functions, their syntax is replacing a definition with `=0`. What is the name of these special member functions and special class.

 - Thse special member functions are called virtual functions and the class is called a WHAT?


