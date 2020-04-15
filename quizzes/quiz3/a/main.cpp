#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>

#include "circle.h"
#include "line.h"

#include "analysis.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;

int main()
{

  vector<Shape *> shapes;

  //! TODO - TASK 1: Create the following shapes and push them back to vector of Shape*
  //! Circle with radius of 2.0
  //! Circle with radius of 3.0
  //! Circle with radius of 4.0
  //!
  //! HINT: refer tutorial week 04 on using new to create pointers to Cicrle
  vector<double> radius;
  radius.push_back(2.0);
  radius.push_back(3.0);
  radius.push_back(4.0);

  for (int i = 0; i < radius.size(); i++)
  {
    cout << "The radius of circle " << i + 1 << " is " << radius[i] << endl;
  }

  shapes.push_back(new Circle(radius[0]));
  shapes.push_back(new Circle(radius[1]));
  shapes.push_back(new Circle(radius[2]));
  // for (auto test : shapes)
  // {
  //   cout << test->getArea() << endl;
  // }
  // shapes.push_back(new Circle(2.0));
  // shapes.push_back(new Circle(3.0));
  // shapes.push_back(new Circle(4.0));

  double x1, y1, x2, y2;
  cout << "Enter first point of line, x and y coordinate separated by spaces: ";
  cin >> x1 >> y1;

  cout << "Enter second point of line, x and y coordinate separated by spaces: ";
  cin >> x2 >> y2;

  //! TODO - TASK 2: Create an object of Class Line, using the two points that form line
  //! Provided in input above (x1 y1) and (x2 y2)
  //!
  //! HINT:  Check line constructors
  //Line(double ax, double ay, double bx, double by);
  Line line(x1, y1, x2, y2);

  //! TODO - TASK 4: Create an object of class Analysis and
  //! Pass the shapes and line to analysis using the approprate member functions
  //!
  //! HINT:  Check Analsis Class
  Analysis test;
  test.setShapes(shapes);
  test.setLine(line);
  test.setRadius(radius);
  cout << "\nAnalysis " << endl;
  std::vector<bool> result = test.intersectsLine();

  for (int i = 0; i < result.size(); i++)
  {
    if (result[i] == true)
    {
      std::cout << shapes.at(i)->getDescription() << " # " << i + 1 << " intersects the line : " << std::boolalpha << result[i] << std::endl;
      std::cout << shapes.at(i)->getDescription() << " # " << i + 1 << "  area = " << shapes[i]->getArea() << "\n"
                << std::endl;
    }
    else
    {
      std::cout << "Line does not intersect " << shapes.at(i)->getDescription() << " # " << i + 1 << std::endl;
    }
  }

  //! TODO - TASK 5: Compute the area of all shapes that intersect the line
  //! using the intersectsLine memberfunction
  //test.intersectsLine();

  for (auto s : shapes)
  {
    delete s;
  }

  return 0;
}
