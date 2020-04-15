Quiz 1
======

Part A
------

**1) QUESTION Attempt to compile the code in [folder a](./a). Why does it fail? (list the reasons)**<br/>
#include "person.h" was missing from main.cpp. Object 'alice' was using the wrong function to set values of name and age. 

**2) TASK Fix the issue so that it compiles.**<br/>
Done 

**3) TASK Make the code more robust, with respect to "sane" values of age.**<br/>
Done 

**4) TASK Create a `crowrd` using a vector container of people, populate it with 3 people.**<br/>
Done

**5) TASK Create a function that greets the oldest member of the `crowd`.**<br/>
Done

**6) TASK Implement a safe guard to make the initialisation of `person` objects easier? (HINT: What special member function is missing in Person?)**<br/>
Done

Part B
------

**1) TASK Modify the file rectangle [rectangle](./b/rectangle.h) so it inherits from the base class of shape is [shape](./b/shape.h)**<br/>Done

**2) TASK Correct the missing access specifiers of base class [shape](./b/shape.h)**<br/>Done

**3) TASK Modify the main [main.cpp](./b/main.cpp) so that it creates a rectangle of size width=5 and height =3.5**<br/>Done

**4) QUESTION If you create a `Rectangle`, which constructor is called, and what does it initialise?**<br/>
Rectangle::Rectangle() is called, initialising the width and height as 0.0
