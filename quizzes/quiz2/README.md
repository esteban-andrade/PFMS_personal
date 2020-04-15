Quiz 2
======

Part A
------

1) TASK: Modify the base class `Shape` file [shape.h](./a/shape.h) such that function getArea() is defined in `Shape`, and the child classes are required to implement this function.  
HINT: Polymorphism and the concept of virtual.

2) TASK: Create a Square and Traingle, and store both of them in a vector of `Shape` pointers

3) TASK: Add `Cicrle` as a child class of `Shape`.
HINT: Use the Circle class developed in Week 03

4) TASK: Create a function that loops through shapes and display their area.
HINT: Think of the function signature.

5) TASK: Write a program that allows the user to specify number of circles and `max_radius`. Create the shapes with random lengths to be capped to `max_length`.


Part B
------

1) TASK: Modify the file [car.h](./a/car.h) so that it inherits behaviour from the base class, [controllerinterface](./a/controllerinterface.h).

2) TASK: Instatiate two objects of type `Car` with different specifications and determine's their top speed.

3) QUESTION: When instantiating an object of the car class, which constructor(s) are caled and in what order.
construction of objects always starts with the base class(in this case the base class is controllerinterface)(Virtual inheritance takes higher preference).Thus it starts from the constructor of controller interface. Then the member fields are constructed, they are initialized in the order they are declared. FInally the class itself(Car) is contructed.
When a derived class object is created using constructors, it is created in the following order:
    1)Virtual base classes are initialized, in the order they appear in the base list.
    2)base classes are initialized, in declaration order.
    3)members are initialized in declaration order (regardless of their order in the initialization list).
    4)body of the constructor is executed.

4) TASK: Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and determines the time difference between the two vehicles reaching zero.

5) QUESTION: Class [controllerinterface](./a/controllerinterface.h) is a special class,  and has special member functions, their syntax is replacing a definition with `=0`. What is the name of these special member functions and special class.
 This is pure virtual method which implies that all the child classes will have to implements this method. And this class will be an abstract class.

