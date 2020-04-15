#include <iostream>
#include <cstring>
#include <algorithm>

//pointers and reference

// double x = 41012;
//     double *ip = &x;
//     double &y =x;
//     double z =1;

//     std::cout<<"print value that ip is pointing to :" <<*ip<<std::endl;
//     std::cout<<"print value of what y is referencing to :" <<y<<std::endl;

//     std::cout<<"Print the value of what ip and y is referencing to :" <<*ip<< " this is y "<<y<<std::endl;

//C Arrays

//   double x[10]; //declaring an array of 10 elements

//   for (int i =0;i < 10 ;i++){
//       x[i]=i;
//       std::cout <<"array\n"<<x[i]<<std::endl;

//   }
//array point

// double x[10]; //declaring an array of 10 elements
// double *p;
// p = x; //*p will point to array x

// std::cout << "adress of x " << p << std::endl;
// for (int i = 0; i < 10; i++)
// {
//     x[i] = i;
//     std::cout << "pointer array " << i << "the reference of arrays is "<< &x[i] << std::endl;
// }

// pointer and loop to initialise an array:
// double x[10]; //declaring an array of 10 elements
// double *p;
// p = x; //*p will point to array x

// std::cout << "adress of x " << p << std::endl;
// for (int i = 0; i < 10; i++)
// {
//     x[i] = i;
//     std::cout << "pointer array " << i << "the reference of arrays is " << &x[i] << std::endl;
// }

// Loops -typescasting

int main()
{
    char x[] = "41012";
    std::string s_x(x);
    std::cout << s_x << std::endl; // converted into string
    int y[strlen(x)];
    for (int i = 0; i < strlen(x); i++)
    {
        y[i] = x[i] - '0';
        std::cout << y[i] << std::endl; //converted into int
    }

    int n, result = 0;

    for (n = 0; n < strlen(x); n++)
    {
        result += y[n];
    }
    std::cout << "This is the sum of the final output:" << result << std::endl; // return vaalue as 8

    int rep=0;
    for (n = 0; n < strlen(x); n++)
    {
        if (y[n] < 2)
        {
            rep++;
        }
      
    }
    std::cout<<"The elements les than 2 are :\n"<< rep << std::endl;
    return 0;
}
