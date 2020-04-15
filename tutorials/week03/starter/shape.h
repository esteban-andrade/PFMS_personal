#ifndef SHAPE_H
#define SHAPE_H

#include <string>

/*!
 *  \ingroup   ac_shapre Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 */
class Shape
{
//!TODO access_specifier:
    Shape();
    void setCentre(double x, double y);
    std::string getDescription();
//! TODO access_specifier:
    std::string description_;//!< description of shape
//! TODO access_specifier:
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
