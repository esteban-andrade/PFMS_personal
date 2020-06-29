#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"
#include "cell.h"

typedef enum
{
  COLINEAR = 0,
  CW = 1,
  CCW = -1
} PointOfOrientation;

typedef struct
{
  double x;
  double y;
} point_t;

typedef struct
{
  point_t origin;
  point_t end;
  bool valid;
} laser_t;

typedef struct
{
  point_t origin;
  point_t endA;
  point_t endB;
  bool valid;
} sonar_t;

typedef struct
{
  point_t a;
  point_t b;
  point_t c;
  point_t d;
  point_t centre;
} cell_t;

typedef struct
{
  point_t p1;
  point_t p2;
} line_t;

class RangerFusion : public RangerFusionInterface
{
public:
  //Default constructor should set all RangerFusion attributes to a default value
  RangerFusion();

  //See rangerfusioninterface.h for more information

  // Accepts container of rangers
  void setRangers(std::vector<RangerInterface *> rangers);

  // Accepts container of cells
  void setCells(std::vector<Cell *> cells);

  // Grab data and fuse
  void grabAndFuseData();

  // Returns a container of raw data range readings
  std::vector<std::vector<double>> getRawRangeData();

  void setOrigin(point_t origin);

private:
  //This is to cater for getRawRangeData (which generates the raw data))
  std::vector<std::vector<double>> data_;

  std::vector<RangerInterface *> rangers_;
  std::vector<Cell *> cells_;

  point_t origin_;
  PointOfOrientation getOrientationPoint(point_t a, point_t b, point_t c);
  bool lineIntersection(line_t l1, line_t l2);
  bool isOnLine(line_t line, point_t point);
  bool updateCellState(Cell &cell, State state);
  int countIntersections(cell_t cell, line_t line);
  bool isPointInTriangle(point_t s , point_t a, point_t b, point_t c);
  double degreesToRadians(double angle);
  bool isPointInCell(Cell &cell, point_t point);
};

#endif // RANGERFUSION_H
