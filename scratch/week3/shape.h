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
public:
    Shape();
    void setCentre(double x, double y);
    std::string getDescription();

protected:
    //! TODO access_specifier:
    std::string description_; //!< description of shape

private:             //! TODO access_specifier:
    double centreX_; //!< X coordinate of centre of shape
    double centreY_; //!< Y coordinate of centre of shape
};

#endif // SHAPE_H
