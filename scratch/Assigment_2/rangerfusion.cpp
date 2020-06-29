#include "rangerfusion.h"

#include <random>
#include <chrono>
#include <cmath>

using std::vector;

RangerFusion::RangerFusion() : rangers_(0), cells_(0) {} //default constructor

void RangerFusion::setRangers(std::vector<RangerInterface *> rangers)
{
    rangers_ = rangers; // assigns the passed vector of ranger that will be used for fusion
}

void RangerFusion::setCells(std::vector<Cell *> cells)
{
    cells_ = cells; // assigns the passed vector of cells that will be used for fusion
}

void RangerFusion::grabAndFuseData()
{
    //vectors from typedef used for analysis
    vector<Laser_t> lasers;
    vector<Sonar_t> sonars;
    for (auto it : rangers_) //iterate all the passed rangers (laser,sonar,sonar)
    {
        vector<double> data;                     // vector that will hold generated data
        SensingMethod sensing_method;            // will hold sensing type for analysis
        data = it->generateData();               // get generated data from iterator
        data_.push_back(data);                   // store generated data
        sensing_method = it->getSensingMethod(); // store sesing type from iterator

        if (sensing_method == CONE) // check if sensing type is sonar
        {
            Sonar_t sonar;
            sonar.origin = origin_;                            // set origin of sonar
            unsigned int field_of_view = it->getFieldOfView(); // get field of view of sonar from iterator
            int offset = it->getOffset();                      //set offset from iterator
            for (auto d : data)
            {
                sonar.valid = ((d <= it->getMaxRange()) && (d >= it->getMinRange())); //checks if the generated data is valid
                double angle;
                angle = (double)offset - ((double)field_of_view / 2.0) + 90; //get the angle for each of the data generated. for first half of cone end
                sonar.endA.x = d * sin(degreesToRadians(angle));             //store first end x coordinante  of sonar cone
                sonar.endA.y = d * cos(degreesToRadians(angle));             //store first end y coordinate of sonar cone

                angle = (double)offset + ((double)field_of_view / 2.0) + 90; // get the angle for the second end of sonar cone
                sonar.endB.x = d * sin(degreesToRadians(angle));             // get second end x coordinate of sonar cone
                sonar.endB.y = d * cos(degreesToRadians(angle));             // get second end y coordinate of sonar cone

                sonars.push_back(sonar); //store sonar generate data into a vector
            }
        }

        if (sensing_method == POINT) // checks if sensing type is laser
        {
            Laser_t laser;
            laser.origin = origin_;
            unsigned int field_of_view = it->getFieldOfView();            // gets field of view from iterator
            unsigned int angular_resolution = it->getAngularResolution(); // gets angular resolution from iterator
            int offset = it->getOffset();                                 //get offset from iterator
            unsigned int samples_taken = 0;                               //sample 0 is first reading taken from laser.
            for (auto d : data)
            {
                laser.valid = ((d <= it->getMaxRange()) && (d >= it->getMinRange()));                 //checks if laser data is valid
                double angle = (double)offset + ((double)samples_taken * (double)angular_resolution); //get the angle based on the number of samples of data taken.
                laser.end.x = d * cos(degreesToRadians(angle));                                       //gets x coordinate of laser end
                laser.end.y = d * sin(degreesToRadians(angle));                                       //gets y coordinate of laser end
                lasers.push_back(laser);                                                              //stores the angles of laser ends
                samples_taken++;                                                                      //increment number if sample of data taken
            }
        }
    }

    for (auto it : cells_) //will iterate in the number of cells
    {

        it->setState(UNKNOWN); //its essential to assume that before analysis the cells status will be unknown
        Cell_t cell;
        it->getCentre(cell.centre.x, cell.centre.y); // get cooodinate of cell
        double side_lenght = it->getSide();          // get the cells side size

        //gets coordinates x and y of first corner of cell
        cell.a.x = cell.centre.x - (side_lenght / 2.0);
        cell.a.y = cell.centre.y + (side_lenght / 2.0);

        //gets coordinates x and y of second corner of cell
        cell.b.x = cell.centre.x + (side_lenght / 2.0);
        cell.b.y = cell.centre.y + (side_lenght / 2.0);

        //gets coordinates x and y of third corner of cell
        cell.c.x = cell.centre.x - (side_lenght / 2.0);
        cell.c.y = cell.centre.y - (side_lenght / 2.0);

        //gets coordinates x and y of fourth corner of cell
        cell.d.x = cell.centre.x + (side_lenght / 2.0);
        cell.d.y = cell.centre.y - (side_lenght / 2.0);

        //iterate all valid laser readings
        for (auto laser : lasers)
        {
            if (laser.valid) //checks if data readings are valid
            {
                Line_t laser_line;
                laser_line.p1 = laser.origin;                             //gets start point of laser data(origin)
                laser_line.p2 = laser.end;                                //gets end point of laser data
                int intersections = countIntersections(cell, laser_line); //counts the number of intersections between a cell and the laser data
                bool is_it_in_cell = isPointInCell(*it, laser.end);       //checks whether the end point of laser is in a cell

                //checks the number of intersections and updates cell status
                if (intersections > 0)
                {
                    updateCellState(*it, FREE);
                }
                if (intersections == 1)
                {
                    updateCellState(*it, OCCUPIED);
                }
                //updates cell status if end point of laser is in a cell
                if (is_it_in_cell)
                {
                    updateCellState(*it, OCCUPIED);
                }
            }
        }

        //iterate in data from sonars
        for (auto sonar : sonars)
        {
            if (sonar.valid) // checks if the data generated from the sonar is valud
            {
                //gets fist line of sonar cone
                Line_t sonar_first_line;
                sonar_first_line.p1 = sonar.origin;
                sonar_first_line.p2 = sonar.endA;

                //gets second line of sonar cone
                Line_t sonar__second_line;
                sonar__second_line.p1 = sonar.origin;
                sonar__second_line.p2 = sonar.endB;

                //check the number of intersection betwen  cell and sonar cone lines
                int intersections = countIntersections(cell, sonar_first_line) + countIntersections(cell, sonar__second_line);

                //gets the upper line of the sonar cone. perpendicular to cone lines
                Line_t upper_line;
                upper_line.p1 = sonar.endA;
                upper_line.p2 = sonar.endB;

                //checks if any the centre point of the cell lies within sonar triangle cone data
                bool cell_inside_sonar = isPointInTriangle(sonar.origin, sonar.endA, sonar.endB, cell.centre);

                //counts the number of intersections and updates cell status
                if (intersections > 0)
                {
                    updateCellState(*it, FREE);
                }
                if (intersections == 1)
                {
                    updateCellState(*it, OCCUPIED);
                }
                //checks if cell is witin sonar triangle cone
                if (cell_inside_sonar)
                {
                    updateCellState(*it, OCCUPIED);
                }
            }
        }
    }
}

//set the origin for all sensors used based on the reference plane used.
void RangerFusion::setOrigin(Point_t point)
{
    point.x = 0;
    point.y = 0;
}

std::vector<std::vector<double>> RangerFusion::getRawRangeData()
{
    return data_; // return vector of raw unfused data
}

//Algorithm used to get the orientation of 3 ordered point
PointOfOrientation RangerFusion::getOrientationPoint(Point_t a, Point_t b, Point_t c)
{
    //reference: http://www.dcs.gla.ac.uk/~pat/52233/slides/Geometry1x1.pdf slide 10th
    double product = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
    // product = 0 --> a, b and c are colinear
    // product > 0 --> Clockwise
    // product < 0 --> Counterclockwise
    if (product == 0)
    {
        return COLINEAR;
    }
    if (product < 0)
    {
        return CCW;
    }
    if (product > 0)
    {
        return CW;
    }
}

//Fuction Created to check if two lines intersect
bool RangerFusion::lineIntersection(Line_t l1, Line_t l2)
{
    //will get the orientation of all lines start and end point
    PointOfOrientation d1 = getOrientationPoint(l1.p1, l1.p2, l2.p1);
    PointOfOrientation d2 = getOrientationPoint(l1.p1, l1.p2, l2.p2);
    PointOfOrientation d3 = getOrientationPoint(l2.p1, l2.p2, l1.p1);
    PointOfOrientation d4 = getOrientationPoint(l2.p1, l2.p2, l1.p2);

    //general case for intersection of two lines
    if ((d1 != d2) && (d3 != d4))
    {
        return true;
    }
    //special cases for line intersections
    return (((d1 == 0) && isOnLine(l1, l2.p1)) ||
            ((d2 == 0) && isOnLine(l1, l2.p2)) ||
            ((d3 == 0) && isOnLine(l2, l1.p1)) ||
            ((d4 == 0) && isOnLine(l2, l1.p2)));
}

//Checks if a given point is in a line
bool RangerFusion::isOnLine(Line_t line, Point_t point)
{
    //based from equation y=mx+c
    double m = (line.p2.y - line.p1.y) / (line.p2.x - line.p1.x); //gets gradient

    if (point.y == (m * point.x)) //checks if point satisfy the condition
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool RangerFusion::updateCellState(Cell &cell, State state)
{
    State current_state = cell.getState(); //gets current state of the cell

    //will update the status  priorities (Occupied and Free)
    switch (state)
    {
    case UNKNOWN:
        break;
    case FREE:                        //checks if cell is free
        if (current_state == UNKNOWN) //double checks if cell status is unknown for safety
        {
            cell.setState(FREE);
            return true;
        }
    case OCCUPIED:
        if (current_state == UNKNOWN) //double checks if cell status is unknown for safety
        {
            cell.setState(OCCUPIED);
            return true;
        }
    }
}
int RangerFusion::countIntersections(Cell_t cell, Line_t line)
{
    int intersections = 0;
    int number_of_sides = 4;
    for (int i = 0; i < number_of_sides; i++) //loops in each 4 sides of the cell
    {
        Line_t line_cell;
        switch (i)
        {
        case 0:
            //check first line AB
            line_cell.p1 = cell.a;
            line_cell.p2 = cell.b;
            break;
        case 1:
            //check second line CD
            line_cell.p1 = cell.c;
            line_cell.p2 = cell.d;
            break;
        case 2:
            //check third line AC
            line_cell.p1 = cell.a;
            line_cell.p2 = cell.c;
            break;
        case 3:
            //check fouth line BD
            line_cell.p1 = cell.b;
            line_cell.p2 = cell.d;
            break;
        }
        if (lineIntersection(line, line_cell)) //check number of intersection between laser line and cell side line
        {
            intersections++; // if true increment number of intersections
        }
    }

    return intersections; // get number of intersections
}

double getAreaTriangle(Point_t a, Point_t b, Point_t c) //will get the area of the sonar triangle cone
{
    double area;
    area = abs((a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2.0);
    return area;
}

//will check if a point in the cell lies inside the sonar triangle cone
bool RangerFusion::isPointInTriangle(Point_t a, Point_t b, Point_t c, Point_t p)
{
    double A = getAreaTriangle(a, b, c); // get area of triangle abc

    double A1 = getAreaTriangle(p, b, c); //get area of triangle pbc

    double A2 = getAreaTriangle(a, p, c); // get area of triangle pac

    double A3 = getAreaTriangle(a, b, p); // get area of PAB

    return (A == A1 + A2 + A3); // checks if sum of A1,A2, A2 =A then point lies on the triangle
}

double RangerFusion::degreesToRadians(double angle)
{
    return ((angle)*M_PI / 180); // return angle in radians for calculations
}

//will analyse if the end point of the laser lies within a cell
bool RangerFusion::isPointInCell(Cell &cell, Point_t point)
{
    Point_t cell_centre;
    cell.getCentre(cell_centre.x, cell_centre.y);
    double side;
    side = cell.getSide();
    //will return true if end points (x,y) of laser are in contact with any cell point
    return (
        (point.x >= cell_centre.x - (side / 2.0)) &&
        (point.x <= cell_centre.x + (side / 2.0)) &&
        (point.y <= cell_centre.y + (side / 2.0)) &&
        (point.y >= cell_centre.y - (side / 2.0)));
}
