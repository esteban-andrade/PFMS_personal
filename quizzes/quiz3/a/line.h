#ifndef LINE_H
#define LINE_H



class Line
{
public:
    Line();
    Line(double gradient, double y_intercept);
    Line(double ax, double ay, double bx, double by);
    void fromPoints(double ax, double ay, double bx, double by);
    void setGradient(double gradient);
    void setYIntercept(double y_intercept);
    bool pointAboveLine(double x, double y);
    double get_dr();
    double getD();

private:
    double gradient_;
    double y_intercept_;
    double ax_; 
    double ay_; 
    double bx_;
    double by_;
    double D_;
    double dr_;

    
};

#endif // LINE_H
