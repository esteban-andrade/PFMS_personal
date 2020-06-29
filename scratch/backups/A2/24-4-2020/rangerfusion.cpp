#include "rangerfusion.h"

#include <random>
#include <chrono>
#include <cmath>

using std::vector;

RangerFusion::RangerFusion()
{
}

void RangerFusion::setRangers(std::vector<RangerInterface *> rangers)
{
    rangers_ = rangers;
}

void RangerFusion::setCells(std::vector<Cell *> cells)
{
    cells_ = cells;
}

void RangerFusion::grabAndFuseData()
{
    vector<laser_t> lasers;
    vector<sonar_t> sonars;
    for (auto it : rangers_)
    {

        vector<double> data;
        SensingMethod sensing_method;
        data = it->generateData();
        data_.push_back(data);

        sensing_method = it->getSensingMethod();

        if (sensing_method == CONE)
        {
            sonar_t sonar;
            sonar.origin = origin_;
            unsigned int field_of_view = it->getFieldOfView();
            int offset = it->getOffset();
            for (auto d : data)
            {
                sonar.valid = ((d <= it->getMaxRange()) && (d >= it->getMinRange()));
                double angle;
                angle = (double)offset - ((double)field_of_view / 2.0) + 90;
                sonar.endA.x = d * sin(degreesToRadians(angle));
                sonar.endA.y = d * cos(degreesToRadians(angle));

                angle = (double)offset + ((double)field_of_view / 2.0) + 90;
                sonar.endB.x = d * sin(degreesToRadians(angle));
                sonar.endB.y = d * cos(degreesToRadians(angle));

                sonars.push_back(sonar);
            }
        }

        if (sensing_method == POINT)
        {
            laser_t laser;
            laser.origin = origin_;
            unsigned int field_of_view = it->getFieldOfView();
            unsigned int angular_resolution = it->getAngularResolution();
            int offset = it->getOffset();
            unsigned int samples_taken = 0;
            for (auto d : data)
            {
                laser.valid = ((d <= it->getMaxRange()) && (d >= it->getMinRange()));
                double angle = (double)offset + ((double)samples_taken * (double)angular_resolution);
                laser.end.x = d * cos(degreesToRadians(angle));
                laser.end.y = d * sin(degreesToRadians(angle));
                lasers.push_back(laser);
                samples_taken++;
            }
        }
    }

    for (auto it : cells_)
    {
        it->setState(UNKNOWN);
        cell_t cell;
        it->getCentre(cell.centre.x, cell.centre.y);
        double side_lenght = it->getSide();
        cell.a.x = cell.centre.x - (side_lenght / 2.0);
        cell.a.y = cell.centre.y + (side_lenght / 2.0);

        cell.b.x = cell.centre.x + (side_lenght / 2.0);
        cell.b.y = cell.centre.y + (side_lenght / 2.0);

        cell.c.x = cell.centre.x - (side_lenght / 2.0);
        cell.c.y = cell.centre.y - (side_lenght / 2.0);

        cell.d.x = cell.centre.x + (side_lenght / 2.0);
        cell.d.y = cell.centre.y - (side_lenght / 2.0);

        for (auto laser : lasers)
        {
            if (laser.valid)
            {
                line_t laser_line;
                laser_line.p1 = laser.origin;
                laser_line.p2 = laser.end;
                int intersections = countIntersections(cell, laser_line);
                bool is_it_in_cell = isPointInCell(*it, laser.end);

                if (intersections > 0)
                {
                    updateCellState(*it, FREE);
                }
                if (intersections == 1)
                {
                    updateCellState(*it, OCCUPIED);
                }
                if (is_it_in_cell)
                {
                    updateCellState(*it, OCCUPIED);
                }
            }
        }
        for (auto sonar : sonars)
        {
            if (sonar.valid)
            {
                line_t sonar_first_line;
                sonar_first_line.p1 = sonar.origin;
                sonar_first_line.p2 = sonar.endA;
                line_t sonar__second_line;
                sonar__second_line.p1 = sonar.origin;
                sonar__second_line.p2 = sonar.endB;
                int intersections = countIntersections(cell, sonar_first_line) + countIntersections(cell, sonar__second_line);

                line_t upper_line;
                upper_line.p1 = sonar.endA;
                upper_line.p2 = sonar.endB;
                bool cell_inside_sonar = isPointInTriangle(sonar.origin, sonar.endA, sonar.endB, cell.centre);
                if (intersections > 0)
                {
                    updateCellState(*it, FREE);
                }
                if (intersections == 1)
                {
                    updateCellState(*it, OCCUPIED);
                }
                if (cell_inside_sonar)
                {
                    updateCellState(*it, OCCUPIED);
                }
            }
        }
    }
}
void RangerFusion::setOrigin(point_t point)
{
    point.x = 0;
    point.y = 0;
}
std::vector<std::vector<double>> RangerFusion::getRawRangeData()
{
    return data_;
}

PointOfOrientation RangerFusion::getOrientationPoint(point_t a, point_t b, point_t c)
{
    double product = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
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
bool RangerFusion::lineIntersection(line_t l1, line_t l2)
{
    PointOfOrientation d1 = getOrientationPoint(l1.p1, l1.p2, l2.p1);
    PointOfOrientation d2 = getOrientationPoint(l1.p1, l1.p2, l2.p2);
    PointOfOrientation d3 = getOrientationPoint(l2.p1, l2.p2, l1.p1);
    PointOfOrientation d4 = getOrientationPoint(l2.p1, l2.p2, l1.p2);
    if ((d1 != d2) && (d3 != d4))
    {
        return true;
    }
    return (((d1 == 0) && isOnLine(l1, l2.p1)) ||
            ((d2 == 0) && isOnLine(l1, l2.p2)) ||
            ((d3 == 0) && isOnLine(l2, l1.p1)) ||
            ((d4 == 0) && isOnLine(l2, l1.p2)));
}
bool RangerFusion::isOnLine(line_t line, point_t point)
{

    double m = (line.p2.y - line.p1.y) / (line.p2.x - line.p1.x);
    if (point.y == (m * point.x))
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
    State current_state = cell.getState();

    switch (state)
    {
    case UNKNOWN:
        break;
    case FREE:
        if (current_state == UNKNOWN)
        {
            cell.setState(FREE);
            return true;
        }
    case OCCUPIED:
        if (current_state == UNKNOWN)
        {
            cell.setState(OCCUPIED);
            return true;
        }
    }
}
int RangerFusion::countIntersections(cell_t cell, line_t line)
{
    int intersections = 0;
    int number_of_sides = 4;
    for (int i = 0; i < number_of_sides; i++)
    {

        line_t line_cell;
        switch (i)
        {
        case 0:
            line_cell.p1 = cell.a;
            line_cell.p2 = cell.b;
            break;
        case 1:
            line_cell.p1 = cell.c;
            line_cell.p2 = cell.d;
            break;
        case 2:
            line_cell.p1 = cell.a;
            line_cell.p2 = cell.c;
            break;
        case 3:
            line_cell.p1 = cell.b;
            line_cell.p2 = cell.d;
            break;
        }
        if (lineIntersection(line, line_cell))
        {
            intersections++;
        }
    }

    return intersections;
}
double getAreaTriangle(point_t a, point_t b, point_t c)
{
    double area;
    area = abs((a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2.0);
    return area;
}
bool RangerFusion::isPointInTriangle(point_t a, point_t b, point_t c, point_t p)
{
    double A = getAreaTriangle(a, b, c); // get area of triangle abc

    double A1 = getAreaTriangle(p, b, c); //get area of triangle pbc

    double A2 = getAreaTriangle(a, p, c); // get area of triangle pac

    double A3 = getAreaTriangle(a, b, p); // get area of PAB

    return (A == A1 + A2 + A3); // checks if sum of A1,A2, A2 =A then point lies on the triangle
}
// bool RangerFusion::isPointInTriangle(point_t s, point_t a, point_t b, point_t c)
// {
//     int as_x = s.x - a.x;
//     int as_y = s.y - a.y;

//     bool s_ab = (b.x - a.x) * as_y - (b.y - a.y) * as_x > 0;

//     if ((c.x - a.x) * as_y - (c.y - a.y) * as_x > 0 == s_ab)
//         return false;

//     if ((c.x - b.x) * (s.y - b.y) - (c.y - b.y) * (s.x - b.x) > 0 != s_ab)
//         return false;

//     return true;
// }

double RangerFusion::degreesToRadians(double angle)
{
    return ((angle)*M_PI / 180);
}

bool RangerFusion::isPointInCell(Cell &cell, point_t point)
{
    point_t cell_centre;
    cell.getCentre(cell_centre.x, cell_centre.y);
    double side;
    side = cell.getSide();
    return (
        (point.x >= cell_centre.x - (side / 2.0)) &&
        (point.x <= cell_centre.x + (side / 2.0)) &&
        (point.y <= cell_centre.y + (side / 2.0)) &&
        (point.y >= cell_centre.y - (side / 2.0)));
}
