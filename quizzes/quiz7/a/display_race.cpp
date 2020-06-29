#include "display_race.h"
#include <cmath>

//Colours of actors within the airspace
static const cv::Scalar CLR_MAP = cv::Scalar(255, 160, 160);
static const cv::Scalar CLR_BSTATION = cv::Scalar(0, 0, 0);
static const cv::Scalar CLR_CAR = cv::Scalar(0, 0, 255);
static const cv::Scalar CLR_TEXT = cv::Scalar(0, 200, 255);

DisplayRace::DisplayRace()
{
    track_ = cv::Mat(MAP_SIZE, MAP_SIZE, CV_8UC3, CLR_MAP);

    cv::namedWindow("track", CV_WINDOW_NORMAL);
    cv::imshow("track", track_);
    cv::waitKey(1); //WaitKey is unavoidable. It will insert a 1ms delay.
}

void DisplayRace::updateDisplay(std::vector<Car> cars)
{

    //Draw the empty race track
    track_ = cv::Mat(MAP_SIZE, MAP_SIZE, CV_8UC3, CLR_MAP);

    //Draw the centre
    int radius = 1;
    int thickness = 1;
    cv::circle(track_, cv::Point(MAP_CENTRE, MAP_CENTRE), radius, CLR_BSTATION, thickness);

    //Draw the race marking
    cv::circle(track_, cv::Point(MAP_CENTRE, MAP_CENTRE), RADIUS + 5, CLR_BSTATION, 3);
    cv::circle(track_, cv::Point(MAP_CENTRE, MAP_CENTRE), RADIUS - 5, CLR_BSTATION, 3);

    std::string info = "centre";
    cv::putText(track_, info, cv::Point(MAP_CENTRE, MAP_CENTRE),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, CLR_TEXT);

    //! @todo
    //! TASK 4
    //!
    //! - Convert distance traveled (odometry) into number of full circles down on track using
    //! RADIUS (which is in header file)
    //! - Computing the remaining angle position (from the full loop)
    double perimeter = 2 * M_PI * RADIUS;
    //double x, y;
    std::vector<double> distance;
    std::vector<double> loops;
    std::vector<double> theta;
    std::vector<double> x;
    std::vector<double> y;
    for (unsigned int i = 0; i < cars.size(); i++)
    {
        distance.push_back(i);
        loops.push_back(i);
        theta.push_back(i);
        x.push_back(i);
        y.push_back(i);
    }
    for (unsigned int i = 0; i < cars.size(); i++)
    {
        distance.at(i) = cars.at(i).getOdometry();
        loops.at(i) = distance.at(i) / perimeter;
        theta.at(i) = distance.at(i) / RADIUS;
        x.at(i) = RADIUS * cos(theta.at(i));
        y.at(i) = RADIUS * sin(theta.at(i));
        // if (cars.at(i).getCurrentSpeed() == 0)
        // {
        //     std::cout << cars.at(i).getMake() << "angle" << theta.at(i) << "\t" << x << "\t" << y << std::endl;
        // }
    }

    //! @todo
    //! TASK 5
    //! Drawing the vehcile position using the cv::circle and model of vehicle on top of the location
    //! using cv::putText
    //!
    for (unsigned int i = 0; i < cars.size(); i++)
    {

        // HINT: Use below to draw the car position and the car name
        //       If you have the remaining angle on the loop
        //       you can compute the position x,y using the radius and the angle
        cv::circle(track_, cv::Point(MAP_CENTRE + x.at(i), MAP_CENTRE + y.at(i)), 1, CLR_CAR, 3);
        cv::putText(track_, cars.at(i).getMake(), cv::Point(MAP_CENTRE + x.at(i), MAP_CENTRE + y.at(i)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, CLR_TEXT);
    }

    cv::imshow("track", track_);
    cv::waitKey(1); //WaitKey is unavoidable. It will insert a 1ms delay.
}
