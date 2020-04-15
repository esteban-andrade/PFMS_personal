#include <iostream>
#include <vector>

#include "rectangle.h"
#include "triangle.h"
#include "shape.h"

using std::cout;
using std::endl;
using std::vector;

int main () {

    Rectangle rectangle;
    rectangle.setHeightWidth(5.0, 3.5);
//    std::cout << "The area of box is " << rectangle.getArea() << std::endl;
//    std::cout << "It is a " << rectangle.getDescription() << std::endl;

    Triangle triangle(3.0, 4.0);
//    std::cout << "The area of our triangle is " << triangle.getArea() << std::endl;
//    std::cout << "It is a " << triangle.getDescription() << std::endl;

    vector<Shape*> shapes;
    shapes.push_back(&rectangle);
    shapes.push_back(&triangle);

    for (auto s : shapes) {
        cout << s->getDescription() << " has area " <<  (*s).getArea() << endl;
    }
}
