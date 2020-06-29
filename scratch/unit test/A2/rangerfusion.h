#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"
#include "cell.h"

/**
 * \details All the elemets used in fusion include many mathematical and trigonometric algorithms
 * \author Esteban Andrade
*/

/**
 * \typedef enum used to check the orientation of a point
 * \enum PointOfOrientation
 * \brief PointOfOrientation this will take the orientation point based on the reference of the origin
 */
typedef enum
{
  COLINEAR = 0, //!< if point orientation is colinear =0
  CW = 1,       //!< if point orientation is clockwise =1
  CCW = -1      //!< if point orientation is counter-clockwise = -1
} PointOfOrientation;

/**
 * \typedef struct used to get the coordinates of a point
 * \struct  Point_t
 * \brief Point_t will take the x and y coordinated of a specific point
*/
typedef struct
{
  double x; //!< x coordinate
  double y; //!< y coordinate
} Point_t;

/**
 * \typedef  struct used to get the point location of start and end of laser
 * \struct Laser_t
 * \brief Laser_t will take the start and end point of the laser reading
*/
typedef struct
{
  Point_t origin; //!< origin (start point) of laser
  Point_t end;    //!< end point of laser
  bool valid;     //!< checks validity of laser reading
} Laser_t;

/**
 * \typedef struct used to get the coordinates of sonar start and both ends of the cone
 * \struct Sonar_t
 * \brief Sonar_t will take the start and end point of the cone reading generated from the sonar
*/
typedef struct
{
  Point_t origin; //!< origin (start point) of sonar cone
  Point_t endA;   //!< end point 1 of cone
  Point_t endB;   //!< end point 2 of cone
  bool valid;     //!< checks validity of sonar reading
} Sonar_t;

/**
 * \typedef struct used to get the coordinates of the cell
 * \struct Cell_t
 * \brief  Cell_t will take all the 4 point of the generated cells
*/
typedef struct
{
  Point_t a;      //!< point 1 of cell (corner 1)
  Point_t b;      //!< point 2 of cell (corner 2)
  Point_t c;      //!< point 3 of cell (corner 3)
  Point_t d;      //!< point 4 of cell (corner 4)
  Point_t centre; //!< centre point  of cell
} Cell_t;

/**
 * \typedef struct used to get the 2 end point of line
 * \struct Line_t
 * \brief Line_t will take both end point of a specific line
*/
typedef struct
{
  Point_t p1; //!< first end point a line
  Point_t p2; //!< second end point a line
} Line_t;

/**
 * \class RangerFusion
 * \brief RangerFusion  class will be use to determine the cell state in reference with the sensors
 * \see rangerfusioninterface.h
*/
class RangerFusion : public RangerFusionInterface
{
  /**
   * \publicsection
  */
public:
  /**
   * Constructor will set the parameters to zero.
   * \note Its neccesary to use set Rangers and Cells in order to fuse it
  */
  RangerFusion();

  /**
   * @brief Accepts container of rangers - as per requirement C1 
   * @note Will set the vectors of rangers that contain the sensors 
   * @param[in] rangers  vector containing (laser and sonars)
   */
  void setRangers(std::vector<RangerInterface *> rangers);

  /**
   * @brief Accepts container of cells - as per requirement C2 
   * @note Will set the cells that are required for fusion the status of these cells will be analysed
   * @param[in] cells vector containing with an specific number of cells
  */
  void setCells(std::vector<Cell *> cells);

  /**
   * @brief Grab data and fuse - as per requirement C3
   * @note  - grabAndFuseData has to be called only once \n - Its neccesary to specify the rangers and cells before fusion
   * \details will perform analysis in cells and determine it's status based on the sensor readings
   * This will iterate to the entire cells and the sensors and determine the state of each cell base on the sensor reading. 
   * The Algorithms that were use include :
   * - Orientation Point: Getting the orientation point  based on a common origin
   * - Line Intersection: This will determine in conjuction with the Orientation Point of both lines intersect. The rationale is to know how many intersection a cell has in order to determine its state
   * - Point on line: By using the equation y=mx+c  we can determine is a specific point belongs in a line.
   * - Counting the Number of intesections: Its neccesary to get the coordinates of the cell and count the number of intersections in reference to a sensor
   * - Cell in Cone: its neccesaty to analyse if any of the cell points lies within a triangle. This is neccesary to analyse if the cell is in the field of view of the sonar
   */
  void grabAndFuseData();

  /**
   * @brief Returns a container of raw data range readings - as per requirement C5 
   * @return 2D vector that containes all the raw unprocessed data from sensors
   * \details This wil return the vector of the data from the last fusion stage. Thus will return all the samples taken by the sensor for the last data fusion process
   */
  std::vector<std::vector<double>> getRawRangeData();

  /**
   * @brief will set the origin of sensors in reference to a plane
   * @param[in] origin typedef struct used to keep track of point location
   */
  void setOrigin(Point_t origin);

  /**
   * \privatesection
  */
private:
  std::vector<std::vector<double>> data_;                                  /**< This is to cater for getRawRangeData (which generates the raw data))*/
  std::vector<RangerInterface *> rangers_;                                 //!< Vector that will contain all the rangers
  std::vector<Cell *> cells_;                                              //!< Vector will contain all the cells
                                                                           /**
  * \note NOTE: methods used for analysis
 */
  Point_t origin_;                                                         //!< Point of reference of origin
  PointOfOrientation getOrientationPoint(Point_t a, Point_t b, Point_t c); /**< Method used to get the orientation with 3 points of reference*/
  bool lineIntersection(Line_t l1, Line_t l2);                             /**< Method used to analyse if two given lines intersect*/
  bool isOnLine(Line_t line, Point_t point);                               /**< Method used to analyse if a point is on a line*/
  bool updateCellState(Cell &cell, State state);                           /**< Method used to pass the cell and analyse and update its status(FREE,OCCUPIED,UNKNOWN)*/
  int countIntersections(Cell_t cell, Line_t line);                        /**< Method used to count the number of intersections between a cell and a line*/
  bool isPointInTriangle(Point_t a, Point_t b, Point_t c, Point_t p);      /**< Method used analyse if a point lies within a triangle (ABC)*/
  double degreesToRadians(double angle);                                   //!< Method used to change the angle to radians
  bool isPointInCell(Cell &cell, Point_t point);                           /**Methos used to analyse if point lies on a given cell*/
};

#endif // RANGERFUSION_H
