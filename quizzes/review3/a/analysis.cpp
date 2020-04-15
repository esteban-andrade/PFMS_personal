#include "analysis.h"
#include <cmath>

#include <iostream>

Analysis::Analysis()
{
}
std::vector<bool> Analysis::intersectsLine()
{
  std::vector<bool> if_in(shapes_.size());
  double x1 = 0;
  double y1 = line_.getYIntercept();

  double x2 = 1;
  double y2 = line_.getGradient()+line_.getYIntercept();

  double dx = x2-x1;
  double dy = y2-y1;

  double dr = sqrt(pow(dx,2)+pow(dy,2));

  double D = x1*y2 - x2*y1;

  for(int i =0; i<shapes_.size();i++)
    {
      if(shapes_.at(i)->getDescription() == "circle")
        {
          double radius = sqrt(shapes_.at(i)->getArea()/M_PI);
          double discriminant = pow(radius,2)*pow(dr,2) - pow(D,2);
          if(discriminant<0)
            {
              if_in[i] = 0;
            }
          else if(discriminant>=0)
            {
              if_in[i] = 1;
            }
        }
    }
  return if_in;

}


//! TODO - TASK 3: Implement the missing function(s) in Analysis Class
//!
//! HINT: Analysis inherits from AnalysisInterface which is an abstract class
//! What type of class is AnalysisInterface, what does this mean?
//!
//! Use the following webiste to assist in developing the code
//! https://mathworld.wolfram.com/Circle-LineIntersection.html
//!
//! BONUS QUESTION (FOR DISCUSSION)
//! At the moment we are implementing check intersect for a Circle
//! as the only question used.
//! If we had a Rectangle shape, how could we differentiate between the approach
//! to intersect for the Rectangle and the Circle?
