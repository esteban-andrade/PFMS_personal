#include "control.h"

using namespace std;

Control::Control()
{
}

void Control::calculate_bogiePos(const std::shared_ptr<Simulator> &sim)
{
    /* locks the calculations until out of scope*/
    lock_guard<mutex> lck(mtx);
    //Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    //Gets the range from base to bogie and range from bogie to friendly
    std::vector<RangeVelocityStamped> D1 = sim->rangeVelocityToBogiesFromBase();   //100ms
    std::vector<RangeBearingStamped> D2 = sim->rangeBearingToBogiesFromFriendly(); //10ms

    distance_bogie_base = D1.at(0).range;
    distance_bogie_friendly = D2.at(0).range;
    timeBogie = D2.at(0).timestamp / 1000;
    /* The following lines represent the equations that work out the two intercepts of two circles, one circle being the radius from bogie to base and another circle from bogie to friendly.
    using algebra and simultaneous equations we get a quadratic formula that can work out the two x positions which ar then subbed into an equation to work out the two y positions*/
    double f;
    double y;
    double a;
    double b;
    double c;

    f = (pow(pose.position.x, 2) + pow(pose.position.y, 2) + pow(distance_bogie_base, 2) - pow(distance_bogie_friendly, 2)) / 2;

    a = pow(pose.position.y, 2) + pow(pose.position.x, 2);
    b = (-2 * f * pose.position.x);
    c = (pow(f, 2) - pow(pose.position.y, 2) * pow(distance_bogie_base, 2));
    // x positions are put into the BogiePos vector
    BogiePos.at(0).position.x = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
    BogiePos.at(1).position.x = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
    // y positions are put into the BogiePos vector
    BogiePos.at(0).position.y = (f - BogiePos.at(0).position.x * pose.position.x) / pose.position.y;
    BogiePos.at(1).position.y = (f - BogiePos.at(1).position.x * pose.position.x) / pose.position.y;
}

void Control::calculate_bogieOrien1(const std::shared_ptr<Simulator> &sim)
{
    // locks the calculations until out of scope
    lock_guard<mutex> lck(mtx);
    // XD1 is the x axis length of a right angle triangle
    double xD1 = BogiePos.at(0).position.x - LastBogiePos.at(0).position.x;
    // this works out the hypotenuse of the triangle
    double hypotenuse1 = sqrt(pow((BogiePos.at(0).position.x - LastBogiePos.at(0).position.x), 2) + pow((BogiePos.at(0).position.y - LastBogiePos.at(0).position.y), 2));
    // the orientation is worked out by working out the angle from the last bogie coordinate to the present, the inverse of cos is used to find the orientation
    BogiePos.at(0).orientation = acos(abs(xD1) / hypotenuse1);

    /* -------Bogie position 1--------*/
    /* if the vector or orientation is pointing towards the first quadrant then orientation is left as is*/
    if (BogiePos.at(0).position.x > LastBogiePos.at(0).position.x && BogiePos.at(0).position.y > LastBogiePos.at(0).position.y)
    {
        BogiePos.at(0).orientation = BogiePos.at(0).orientation;
    }
    /* if the vector or orientation is pointing towards the second quadrant then orientation is adjusted accordingly */
    if (BogiePos.at(0).position.x < LastBogiePos.at(0).position.x && BogiePos.at(0).position.y > LastBogiePos.at(0).position.y)
    {
        BogiePos.at(0).orientation = 3.14 - BogiePos.at(0).orientation;
    }

    /* if the vector or orientation is pointing towards the third quadrant then orientation is adjusted accordingly */
    if (BogiePos.at(0).position.x < LastBogiePos.at(0).position.x && BogiePos.at(0).position.y < LastBogiePos.at(0).position.y)
    {
        BogiePos.at(0).orientation = 3.14 + BogiePos.at(0).orientation;
    }
    /* if the vector or orientation is pointing towards the fourth quadrant then orientation is adjusted accordingly */
    if (BogiePos.at(0).position.x > LastBogiePos.at(0).position.x && BogiePos.at(0).position.y < LastBogiePos.at(0).position.y)
    {
        BogiePos.at(0).orientation = 6.28 - BogiePos.at(0).orientation;
    }
}

void Control::calculate_bogieOrien2(const std::shared_ptr<Simulator> &sim)
{
    // locks the calculations until out of scope
    lock_guard<mutex> lck(mtx);
    // XD2 is the x axis length of a right angle triangle
    double xD2 = BogiePos.at(1).position.x - LastBogiePos.at(1).position.x;
    // this works out the hypotenuse of the triangle
    double hypotenuse2 = sqrt(pow((BogiePos.at(1).position.x - LastBogiePos.at(1).position.x), 2) + pow((BogiePos.at(1).position.y - LastBogiePos.at(1).position.y), 2));
    // the orientation is worked out by working out the angle from the last bogie coordinate to the present, the inverse of cos is used to find the orientation
    BogiePos.at(1).orientation = acos(abs(xD2) / hypotenuse2);

    /* -------Bogie position 2--------*/
    /* if the vector or orientation is pointing towards the first quadrant then orientation is left as is*/
    if (BogiePos.at(1).position.x > LastBogiePos.at(1).position.x && BogiePos.at(1).position.y > LastBogiePos.at(1).position.y)
    {
        BogiePos.at(1).orientation = BogiePos.at(1).orientation;
    }
    /* if the vector or orientation is pointing towards the second quadrant then orientation is adjusted accordingly */
    if (BogiePos.at(1).position.x < LastBogiePos.at(1).position.x && BogiePos.at(1).position.y > LastBogiePos.at(1).position.y)
    {
        BogiePos.at(1).orientation = 3.14 - BogiePos.at(1).orientation;
    }

    /* if the vector or orientation is pointing towards the third quadrant then orientation is adjusted accordingly */
    if (BogiePos.at(1).position.x < LastBogiePos.at(1).position.x && BogiePos.at(1).position.y < LastBogiePos.at(1).position.y)
    {
        BogiePos.at(1).orientation = 3.14 + BogiePos.at(1).orientation;
    }
    /* if the vector or orientation is pointing towards the fourth quadrant then orientation is adjusted accordingly */
    if (BogiePos.at(1).position.x > LastBogiePos.at(1).position.x && BogiePos.at(1).position.y < LastBogiePos.at(1).position.y)
    {
        BogiePos.at(1).orientation = 6.28 - BogiePos.at(1).orientation;
    }
}

void Control::getLastBogiePos(const std::shared_ptr<Simulator> &sim)
{
    // locks the function until out of scope
    lock_guard<mutex> lck(mtx);
    // lets the LastBogiePos vector = BogiePos vector, this function is called after the sleep to remember the last position
    LastBogiePos.at(0).position.x = BogiePos.at(0).position.x;
    LastBogiePos.at(0).position.y = BogiePos.at(0).position.y;

    LastBogiePos.at(1).position.x = BogiePos.at(1).position.x;
    LastBogiePos.at(1).position.y = BogiePos.at(1).position.y;
}

void Control::predict_futurePoint1(const std::shared_ptr<Simulator> &sim)
{
    // locks the function until out of scope
    lock_guard<mutex> lck(mtx);
    // function adds or minus a number to the x and y depending on the orientation of the bogie to "predict" where the bogie will end up. it is a fixed distance ahead of the bogie depending on the number
    /*looking at the first quadrant*/
    if (BogiePos.at(0).position.x > LastBogiePos.at(0).position.x && BogiePos.at(0).position.y > LastBogiePos.at(0).position.y)
    {
        PredictPos.at(0).position.x = BogiePos.at(0).position.x + number;
        PredictPos.at(0).position.y = BogiePos.at(0).position.y + number;
    }
    /*looking at the second quadrant*/
    if (BogiePos.at(0).position.x < LastBogiePos.at(0).position.x && BogiePos.at(0).position.y > LastBogiePos.at(0).position.y)
    {
        PredictPos.at(0).position.x = BogiePos.at(0).position.x - number;
        PredictPos.at(0).position.y = BogiePos.at(0).position.y + number;
    }
    /*looking at the third quadrant*/
    if (BogiePos.at(0).position.x < LastBogiePos.at(0).position.x && BogiePos.at(0).position.y < LastBogiePos.at(0).position.y)
    {
        PredictPos.at(0).position.x = BogiePos.at(0).position.x - number;
        PredictPos.at(0).position.y = BogiePos.at(0).position.y - number;
    }
    /*looking at the fourth quadrant*/
    if (BogiePos.at(0).position.x > LastBogiePos.at(0).position.x && BogiePos.at(0).position.y < LastBogiePos.at(0).position.y)
    {
        PredictPos.at(0).position.x = BogiePos.at(0).position.x + number;
        PredictPos.at(0).position.y = BogiePos.at(0).position.y - number;
    }
}

void Control::predict_futurePoint2(const std::shared_ptr<Simulator> &sim)
{
    // locks the function until out of scope
    lock_guard<mutex> lck(mtx);
    // function adds or minus a number to the x and y depending on the orientation of the bogie to "predict" where the bogie will end up. it is a fixed distance ahead of the bogie depending on the number
    /*looking at the first quadrant*/
    if (BogiePos.at(0).position.x > LastBogiePos.at(0).position.x && BogiePos.at(0).position.y > LastBogiePos.at(0).position.y)
    {
        PredictPos.at(0).position.x = BogiePos.at(0).position.x + number;
        PredictPos.at(0).position.y = BogiePos.at(0).position.y + number;
    }
    /*looking at the second quadrant*/
    if (BogiePos.at(0).position.x < LastBogiePos.at(0).position.x && BogiePos.at(0).position.y > LastBogiePos.at(0).position.y)
    {
        PredictPos.at(0).position.x = BogiePos.at(0).position.x - number;
        PredictPos.at(0).position.y = BogiePos.at(0).position.y + number;
    }
    /*looking at the third quadrant*/
    if (BogiePos.at(0).position.x < LastBogiePos.at(0).position.x && BogiePos.at(0).position.y < LastBogiePos.at(0).position.y)
    {
        PredictPos.at(0).position.x = BogiePos.at(0).position.x - number;
        PredictPos.at(0).position.y = BogiePos.at(0).position.y - number;
    }
    /*looking at the fourth quadrant*/
    if (BogiePos.at(0).position.x > LastBogiePos.at(0).position.x && BogiePos.at(0).position.y < LastBogiePos.at(0).position.y)
    {
        PredictPos.at(0).position.x = BogiePos.at(0).position.x + number;
        PredictPos.at(0).position.y = BogiePos.at(0).position.y - number;
    }
}

void Control::calculate_BogieFriendlyAngle1(const std::shared_ptr<Simulator> &sim)
{
    // locks the function until out of scope
    lock_guard<mutex> lck(mtx);
    //Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    // sets the decimal place
    cout << setprecision(2) << fixed;
    // the x distance between the bogie and friendly
    double Dx1 = BogiePos.at(0).position.x - pose.position.x;
    // the angle from friengly to bogie is worked out by the right angle triangle of bogie and friendly, the inverse of cos is used to find the angle
    theta1 = acos(abs(Dx1) / distance_bogie_friendly);

    // the angle (theta1) represents the angle the friendly needs to turn to be pointing towards the bogie
    /* -------Bogie position 1--------*/
    /* if the bogie is in the first quadrant with respect to friendly theta1 = theta1 */
    if (BogiePos.at(0).position.x > pose.position.x && BogiePos.at(0).position.y > pose.position.y)
    {
        theta1 = theta1;
    }
    /* if the bogie is in the second quadrant with respect to friendly theta1 is adjusted accordingly */
    if (BogiePos.at(0).position.x < pose.position.x && BogiePos.at(0).position.y > pose.position.y)
    {
        theta1 = 3.14 - theta1;
    }

    /* if the bogie is in the third quadrant with respect to friendly theta1 is adjusted accordingly */
    if (BogiePos.at(0).position.x < pose.position.x && BogiePos.at(0).position.y < pose.position.y)
    {
        theta1 = 3.14 + theta1;
    }
    /* if the bogie is in the fourth quadrant with respect to friendly theta1 is adjusted accordingly */
    if (BogiePos.at(0).position.x > pose.position.x && BogiePos.at(0).position.y < pose.position.y)
    {
        theta1 = 6.28 - theta1;
    }
}

void Control::calculate_BogieFriendlyAngle2(const std::shared_ptr<Simulator> &sim)
{
    // locks the function until out of scope
    lock_guard<mutex> lck(mtx);
    //Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    // the x distance between the bogie and friendly
    double Dx2 = BogiePos.at(0).position.x - pose.position.x;
    // the angle from friengly to bogie is worked out by the right angle triangle of bogie and friendly, the inverse of cos is used to find the angle
    theta2 = acos(abs(Dx2) / distance_bogie_friendly);

    // the angle (theta2) represents the angle the friendly needs to turn to be pointing towards the bogie
    /*------------Bogie position 2---------*/
    /* if the bogie is in the first quadrant with respect to friendly theta2 = theta2 */
    if (BogiePos.at(0).position.x > pose.position.x && BogiePos.at(0).position.y > pose.position.y)
    {
        theta2 = theta2;
    }
    /* if the bogie is in the second quadrant with respect to friendly theta2 is adjusted accordingly */
    if (BogiePos.at(0).position.x < pose.position.x && BogiePos.at(0).position.y > pose.position.y)
    {
        theta2 = 3.14 - theta2;
    }

    /* if the bogie is in the third quadrant with respect to friendly theta1 is adjusted accordingly */
    if (BogiePos.at(0).position.x < pose.position.x && BogiePos.at(0).position.y < pose.position.y)
    {
        theta2 = 3.14 + theta2;
    }
    /* if the bogie is in the fourth quadrant with respect to friendly theta1 is adjusted accordingly */
    if (BogiePos.at(0).position.x > pose.position.x && BogiePos.at(0).position.y < pose.position.y)
    {
        theta2 = 6.28 - theta2;
    }
}

void Control::getTime(const std::shared_ptr<Simulator> &sim)
{
    // locks the function until out of scope
    lock_guard<mutex> lck(mtx);
    // LastTime vector = to the current time, this will be called after the sleep to remember the previous time for the last bogie position
    LastTime.at(0) = timeBogie;
}

void Control::calculate_bogieVelocity(const std::shared_ptr<Simulator> &sim)
{
    // locks the function until out of scope
    lock_guard<mutex> lck(mtx);
    // distance formula between bogie position 1 and its corresponding previous position
    double distance1 = sqrt(pow((BogiePos.at(0).position.x - LastBogiePos.at(0).position.x), 2) + pow((BogiePos.at(0).position.y - LastBogiePos.at(0).position.y), 2));
    // distance formula between bogie position 2 and its corresponding previous position
    double distance2 = sqrt(pow((BogiePos.at(0).position.x - LastBogiePos.at(0).position.x), 2) + pow((BogiePos.at(0).position.y - LastBogiePos.at(0).position.y), 2));
    // working out the time between the present and last positon
    long time = timeBogie - LastTime.at(0);
    // speed formula to work out velocity
    velocity1 = distance1 / time;
    velocity2 = distance2 / time;
}

void Control::followBogie1(const std::shared_ptr<Simulator> &sim)
{
    //Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    // sets the velocity for turning
    double velocity = 50;
    // sets the angular velocity for turning, calculated so its limit is only 5.9G
    double omega = 57.879 / velocity;
    // if the bogie position in the x and y goes beyond 2500 or -2500, friendly aircraft will run in circles to avoid following the bogie out of bounds
    if (BogiePos.at(0).position.x > sim->AIRSPACE_SIZE / 2 ||
        BogiePos.at(0).position.y > sim->AIRSPACE_SIZE / 2 ||
        BogiePos.at(0).position.x > -sim->AIRSPACE_SIZE / 2 ||
        BogiePos.at(0).position.y > -sim->AIRSPACE_SIZE / 2)
    {
        sim->controlFriendly(velocity, omega);
    }
    // aircraft runs to ensure watchdog timer is fed
    sim->controlFriendly(velocity, omega);
    //situation 1 is when the angle between bogie from friendly is larger than the orientation of friendly aircraft, this determines if the friendly aircraft needs to turn CW or CCW
    /* Situation 1 */
    if (theta1 > pose.orientation)
    {
        // the difference the angle it needs to turn to be looking at the bogie
        Difference = theta1 - pose.orientation;
        // if difference is more than 180 then it is best to turn CW
        if (Difference > 3.14)
        {
            Difference = (6.28 - theta1) + pose.orientation;
            // if the difference is greater than the tolerance it will continue to rotate
            if (Difference > 0.1)
            {
                sim->controlFriendly(velocity, -omega);
            }
            // if the difference is less than the tolerance it will fly straight
            else if (Difference < 0.1)
            {
                sim->controlFriendly(1000, 0);
            }
        }
        else
        {
            // if the difference is less than 180 it will continue to rotate CCW
            if (Difference > 0.1)
            {
                sim->controlFriendly(velocity, omega);
            }
            // if the difference is less than the tolerance it will fly straight
            else if (Difference < 0.1)
            {
                sim->controlFriendly(1000, 0);
            }
        }
    }
    //situation 2 is when the angle between bogie from friendly is smaller than the orientation of friendly aircraft, this determines if the friendly aircraft needs to turn CW or CCW
    /* Situation 2 */
    if (pose.orientation > theta1)
    {
        // the difference the angle it needs to turn to be looking at the bogie
        Difference = pose.orientation - theta1;
        // if difference is more than 180 then it is best to turn CCW
        if (Difference > 3.14)
        {
            Difference = (6.28 - pose.orientation) + theta1;
            // if the difference is greater than the tolerance it will continue to rotate
            if (Difference > 0.1)
            {
                sim->controlFriendly(velocity, omega);
            }
            // if the difference is less than the tolerance it will fly straight
            else if (Difference < 0.1)
            {
                sim->controlFriendly(1000, 0);
            }
        }
        else
        {
            // if the difference is less than 180 it will continue to rotate CW
            if (Difference > 0.1)
            {
                sim->controlFriendly(velocity, -omega);
            }
            // if the difference is less than the tolerance it will fly straight
            else if (Difference < 0.1)
            {
                sim->controlFriendly(1000, 0);
            }
        }
    }
}

void Control::followBogie2(const std::shared_ptr<Simulator> &sim)
{
    //Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    // sets the velocity for turning
    double velocity = 50;
    // sets the angular velocity for turning, calculated so its limit is only 5.9G
    double omega = 57.879 / velocity;
    // if the bogie position in the x and y goes beyond 2500 or -2500, friendly aircraft will run in circles to avoid following the bogie out of bounds
    if (BogiePos.at(0).position.x > sim->AIRSPACE_SIZE / 2 ||
        BogiePos.at(0).position.y > sim->AIRSPACE_SIZE / 2 ||
        BogiePos.at(0).position.x > -sim->AIRSPACE_SIZE / 2 ||
        BogiePos.at(0).position.y > -sim->AIRSPACE_SIZE / 2)
    {
        sim->controlFriendly(velocity, omega);
    }
    // aircraft runs to ensure watchdog timer is fed
    sim->controlFriendly(velocity, omega);
    //situation 1 is when the angle between bogie from friendly is larger than the orientation of friendly aircraft, this determines if the friendly aircraft needs to turn CW or CCW
    /* Situation 1 */
    if (theta2 > pose.orientation)
    {
        // the difference the angle it needs to turn to be looking at the bogie
        Difference = theta2 - pose.orientation;
        // if difference is more than 180 then it is best to turn CW
        if (Difference > 3.14)
        {
            Difference = (6.28 - theta2) + pose.orientation;
            // if the difference is greater than the tolerance it will continue to rotate
            if (Difference > 0.1)
            {
                sim->controlFriendly(velocity, -omega);
            }
            // if the difference is less than the tolerance it will fly straight
            else if (Difference < 0.1)
            {
                sim->controlFriendly(1000, 0);
            }
        }
        else
        {
            // if the difference is less than 180 it will continue to rotate CCW
            if (Difference > 0.1)
            {
                sim->controlFriendly(velocity, omega);
            }
            // if the difference is less than the tolerance it will fly straight
            else if (Difference < 0.1)
            {
                sim->controlFriendly(1000, 0);
            }
        }
    }
    //situation 2 is when the angle between bogie from friendly is smaller than the orientation of friendly aircraft, this determines if the friendly aircraft needs to turn CW or CCW
    /* Situation 2 */
    if (pose.orientation > theta2)
    {
        // the difference the angle it needs to turn to be looking at the bogie
        Difference = pose.orientation - theta2;
        // if difference is more than 180 then it is best to turn CCW
        if (Difference > 3.14)
        {
            Difference = (6.28 - pose.orientation) + theta2;
            // if the difference is greater than the tolerance it will continue to rotate
            if (Difference > 0.1)
            {
                sim->controlFriendly(velocity, omega);
            }
            // if the difference is less than the tolerance it will fly straight
            else if (Difference < 0.1)
            {
                sim->controlFriendly(1000, 0);
            }
        }
        else
        {
            // if the difference is less than 180 it will continue to rotate CW
            if (Difference > 0.1)
            {
                sim->controlFriendly(velocity, -omega);
            }
            // if the difference is less than the tolerance it will fly straight
            else if (Difference < 0.1)
            {
                sim->controlFriendly(1000, 0);
            }
        }
    }
}

void Control::control_Friendly(const std::shared_ptr<Simulator> &sim)
{
    //could not figure out a way to swap between the two points, for testing run followingBogie1 to follow bogie position 1 or run followingBogie2 to follow bogie position 2
    followBogie1(sim);
    //followBogie2(sim);
}

void Control::printscreen(const std::shared_ptr<Simulator> &sim)
{
    //Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    cout << "--------------------------------------------------------------------------" << endl;
    cout << "[" << sim->elapsed() / 1000 << "s]" << endl;
    cout << "Friendly {x, y, orientation}:" << endl;
    cout << "  - x: " << pose.position.x << " m" << endl;
    cout << "  - y: " << pose.position.y << " m" << endl;
    cout << "  - orient: " << pose.orientation << " radians" << endl
         << endl;

    cout << "Range from bogie to friendly " << distance_bogie_friendly << "m" << endl;
    cout << "Range from base to bogie " << distance_bogie_base << "m" << endl;
    cout << "Bogie Position 1" << endl;
    cout << "Predicted Position, Current Position, Previous Position" << endl;
    cout << "[" << PredictPos.at(0).position.x << ", " << PredictPos.at(0).position.y << "] "
         << "[" << BogiePos.at(0).position.x << ", " << BogiePos.at(0).position.y << "] "
         << "[" << LastBogiePos.at(0).position.x << ", " << LastBogiePos.at(0).position.y << "]" << endl;

    cout << "Bogie Position 2" << endl;
    cout << "Predicted Position, Current Position, Previous Position" << endl;
    cout << "[" << PredictPos.at(1).position.x << ", " << PredictPos.at(1).position.y << "] "
         << "[" << BogiePos.at(1).position.x << ", " << BogiePos.at(1).position.y << "] "
         << "[" << LastBogiePos.at(1).position.x << ", " << LastBogiePos.at(1).position.y << "]" << endl;

    cout << "Bogie orientation 1 = " << BogiePos.at(0).orientation << " radians" << endl;
    cout << "Bogie orientation 2 = " << BogiePos.at(1).orientation << " radians" << endl;

    cout << "Bogie 1 velocity = " << velocity1 << " m/s" << endl;
    cout << "Bogie 2 velocity = " << velocity2 << " m/s" << endl;
}
