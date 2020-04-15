#include <iostream>
#include <vector>
#include <chrono>

#include "circle.h"
#include "line.h"

#include "analysis.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;


int main () {

  vector<Shape*> shapes;

  //! TODO - TASK 1: Create the following shapes and push them back to vector of Shape*
  //! Circle with radius of 2.0
  //! Circle with radius of 3.0
  //! Circle with radius of 4.0
  //!
  //! HINT: refer tutorial week 04 on using new to create pointers to Cicrle


  //TASK 1
  for(int i =2; i<5;i++)
    {
      shapes.push_back(new Circle(i));
    }


  double x1, y1 , x2, y2;
  cout << "Enter first point of line, x and y coordinate separated by spaces: ";
  cin >> x1 >> y1;

  cout << "Enter second point of line, x and y coordinate separated by spaces: ";
  cin >> x2 >> y2;

  //! TODO - TASK 2: Create an object of Class Line, using the two points that form line
  //! Provided in input above (x1 y1) and (x2 y2)
  //!
  //! HINT:  Check line constructors

  Line line(x1,y1,x2,y2);


  //! TODO - TASK 4: Create an object of class Analysis and
  //! Pass the shapes and line to analysis using the approprate member functions
  //!
  //! HINT:  Check Analsis Class

  Analysis analysis;
  analysis.setShapes(shapes);
  analysis.setLine(line);

  //! TODO - TASK 5: Compute the area of all shapes that intersect the line
  //! using the intersectsLine memberfunction
  vector<bool> ifIntersect = analysis.intersectsLine();
  double sum = 0;
  for(int i =0 ; i < shapes.size(); i++)
    {
      if(ifIntersect.at(i))
        {
          sum += shapes.at(i)->getArea();
        }
    }

  cout << sum << " < The sum of all the intersecting circles" << endl;

  for (auto s : shapes) {
      delete s;
    }

}


