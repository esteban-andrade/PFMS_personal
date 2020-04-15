Quiz 1
======

Part A
------

1) QUESTION Attempt to compile the code in [folder a](./a). Why does it fail? (list the reasons)
- Header file not included in main function
- Method not properly defined in main function
- Methods in header file and person cpp does not match definition

2) TASK Fix the issue so that it compiles.
Header file Added and method properly changed in main

3) TASK Make the code more robust, with respect to "sane" values of age.
condition added that it will set to zero values that will are negative or exceed limit.

4) TASK Create a `crowrd` using a vector container of people, populate it with 3 people.

5) TASK Create a function that greets the oldest member of the `crowd`.

6) TASK Implement a safe guard to make the initialisation of `person` objects easier? (HINT: What special member function is missing in Person?)
constructor used.

Part B
------

1) TASK Modify the file rectangle [rectangle](./b/rectangle.h) so it inherits from the base class of shape is [shape](./b/shape.h)

2) TASK Correct the missing access specifiers of base class [shape](./b/shape.h)

3) TASK Modify the main [main.cpp](./b/main.cpp) so that it creates a rectangle of size width=5 and height =3.5

4) QUESTION If you create a `Rectangle`, which constructor is called, and what does it initialise?
if we call Rectangle we will call the rectangle constructor. Which is the default constructor  with values of width & height of 0 , and description of square.
Also it creates an instance of both parent and child classes.


