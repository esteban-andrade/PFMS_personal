#include "analysis.h"

#include <iostream>
#include <cmath>

Analysis::Analysis()
{
}

//! TODO - TASK 3: Implement the missing function(s) in Analysis Class
//!
//! HINT: Analysis inherits from AnalysisInterface which is an abstract class
//! What type of class is AnalysisInterface, what does this mean?
//!
// void Analysis::setShapes(std::vector<Shape*> shapes){
// shapes_ =  shapes;
// }

// void Analysis::setLine(Line line){
// line_= line;
// }

std::vector<bool> Analysis::intersectsLine()
{
    std::vector<bool> result;

    double area = 0;
    double discriminant = 0;

    // for (auto test : shapes_)
    // {
    //     area += test->getArea();

    //     if (discriminant >= 0)
    //     {
    //         result.push_back(true);
    //     }
    // }
    // for (int i = 0; i < result.size(); i++)
    // {
    //     std::cout << std::boolalpha << result[i] << std::endl;
    // }

    for (int i = 0; i < shapes_.size(); i++)
    {
        double radius_squared = pow(radius_[i], 2);
        double dr_squared = pow(line_.get_dr(), 2);
        double D_squared = pow(line_.getD(), 2);
        discriminant = radius_squared * dr_squared - D_squared;
        // std::cout << radius_squared << std::endl;
        // std::cout << dr_squared << std::endl;
        // std::cout << D_squared << std::endl;
        // std::cout << discriminant << std::endl;

        //std::cout << "The discriminant at " << shapes_.at(i)->getDescription() << " # " << i + 1 << " : " << discriminant  << std::endl;
        if (discriminant >= 0)
        {
            result.push_back(true);
            //std::cout << shapes_.at(i)->getDescription() << " # " << i + 1 << "  area = " << shapes_[i]->getArea() << std::endl;
        }
        else if (discriminant < 0)
        {
            result.push_back(false);
            //std::cout << "Line does not intersect circle # " << i + 1 << std::endl;
        }
    }
    //std::cout << "\nResults conditions :" << std::endl;
    // for (int i = 0; i < result.size(); i++)
    // {
    //     if (result[i] == true)
    //     {
    //         std::cout << shapes_.at(i)->getDescription() << " # " << i + 1 << " intersects the line : " << std::boolalpha << result[i] << std::endl;
    //     }
    // }

    return result;
}
//! Use the following webiste to assist in developing the code
//! https://mathworld.wolfram.com/Circle-LineIntersection.html
//!
//! BONUS QUESTION (FOR DISCUSSION)
//! At the moment we are implementing check intersect for a Circle
//! as the only question used.
//! If we had a Rectangle shape, how could we differentiate between the approach
//! to intersect for the Rectangle and the Circle?
