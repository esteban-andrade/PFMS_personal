Quiz 0
======

Part A
------
1) Attempt to compile the code in [folder a](./a). Why does the code fail to compile?
Because funtions printArray calls the same definition. Its necessaty to only use one.

2) Fix the compilation error. Why does a segmentation fault (segfault) occur?
This is caused by accessing memory that does not belong to you. Am  example could be a null pointer. In this case is because the array is contrain. Therefore we might be able to use vectors instead or to use different arrays and then concatonate them. Look in array vector for example.

3) Modify the code so it will run
The code was modifies so it can run successfully. It was designed that it will create a new array. And the concatonate both arrays into  one that will give the final result

4) Create a function that prints only the elements of the array that are larger than : mean + one standard deviation
This was done using cmath library

5) Create a function that assigns elements of array x to a vector named `vec` (HINT: decide on the correct type)
There is a funtion that will take the final array and covert its elements to a vector.

Part B
------
1) Look at the code in [folder b](./b). Implement the methods of the Sample class in the [sample.cpp](./b/sample.cpp) file based on the definition provided in [sample.h](./b/sample.h)

2) Make an executable that creates an object `sample` of `Sample` class and then obtains the value of parameter `value_` in this object.
This was done by creating an object from the method itself that will return the value.
This object was created using the provided constructor that will assign a value. Similarly the value is later changed using the method.

3) What do we call functions of a class?
 function in a class are called methods. They are used as part of an object meta-model.

4) What access specifiers are used?
Are features on object oriented programming known as Data Hiding. Thus, they are used to set the accessibility of the class members. These Define how the members (attibutes and methods) of a class can be accessed. There are 3 access specifiers:
PUBLIC --> members are accesible outside of the class
PRIVARE --> members cant be accessed or view from outside of the class
PROTECTED --> members cant be accesed from outside the class. However, they can be accessed with inheritance.


5) What do we call variable `value_` in the `Sample` class?
    This is an Public attribute. Which is of type int, that later is used to pass values to methods and constructors.

