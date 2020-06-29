#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <random>
#include <algorithm>
#include "laser.h"
#include "sonar.h"
#include "rangerfusion.h"
#include "cell.h"

//declaration of std:: for better readability
using std::cin;
using std::cout;
using std::endl;
using std::vector;

//functions declaration
void initializationLaser(Laser *);
void initializationSonarOne(Sonar *);
void initializationSonarTwo(Sonar *);
void configureLaser(Laser *);
void configureSonarOne(Sonar *);
void configureSonarTwo(Sonar *);
void sensorsSettingsSummary(Laser *, Sonar *, Sonar *);
void cellsMapSize(vector<Cell *> &, Laser *, Sonar *, Sonar *);

int main()
{
    //Objects creation
    Laser laser;
    Sonar sonar_1;
    Sonar sonar_2;

    cout << "Starting Sensors" << endl;

    //Sensors initialization
    initializationLaser(&laser);
    initializationSonarOne(&sonar_1);
    initializationSonarTwo(&sonar_2);

    //Sensor configuration
    configureLaser(&laser);
    configureSonarOne(&sonar_1);
    configureSonarTwo(&sonar_2);

    //displaying sensor summary configuration
    sensorsSettingsSummary(&laser, &sonar_1, &sonar_2);

    vector<Cell *> cells; // create vector of pointers to cell
    cells.clear();        //ensures that the vector is empty
    int user_cell_input;
    cout << "Please Enter the number of Cells that need to be created : " << endl;
    while (!(cin >> user_cell_input) || user_cell_input <= 0) // check that input is valid
    {
        cin.clear();
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        cout << "Invalid input, Try again: ";
    }

    cells.reserve(user_cell_input + 1); // will reserve vector size for better memory management

    for (int i; i < user_cell_input; i++) // create required cells based on input
    {
        cells.push_back(new Cell()); //store new created cells
    }

    cout << "Number of cells created : " << cells.size() << endl;
    cellsMapSize(cells, &laser, &sonar_1, &sonar_2);                  // will remap cell based on sensor maximun range. Thus map limit will be sensor max range
    RangerFusion fusion;                                              // create fusion
    vector<RangerInterface *> rangers = {&laser, &sonar_1, &sonar_2}; // pass objects to vector of ranger interface for fusion
    fusion.setRangers(rangers);                                       // set ranger for fusion
    fusion.setCells(cells);                                           // set cells for fusion
    Point_t origin;                                                   //set origin points
    origin.x = 0;                                                     //x coordinate = 0
    origin.y = 0;                                                     //y coordinate = 0
    fusion.setOrigin(origin);                                         //set origin for fusion

    cout << "\n Starting Fusion : " << endl;
    int index = 0;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));       // set for fusion every second
        fusion.grabAndFuseData();                                   //fusion data
        vector<vector<double>> raw_data = fusion.getRawRangeData(); //return vector of raw unfused data
        index++;
        cout << "Sample" << std::setw(4) << index << ":";
        for (int i = 0; i < cells.size(); i++)
        {
            cout << std::setw(4) << cells.at(i)->getState(); // get cells state
        }
        cout << endl;
    }
    return 0;
}

/*Functions implementation*/

void initializationLaser(Laser *laser) //Laser initialization
{
    cout << "\nLaser Model : " << laser->getModel() << endl;
    cout << "Laser Field of view : " << laser->getFieldOfView() << " degrees" << endl;
    cout << "Laser Configuration # " << laser->getLaserConfiguration() << endl;
    cout << "Laser Default Angular Resolution : " << laser->getAngularResolution() << " degrees" << endl;
    cout << "Sensing Type: " << laser->getSensingMethod() << endl;
    cout << "Number of Samples : " << laser->getNumberSamples() << endl;
    cout << "Laser Max Distance : " << laser->getMaxRange() << " m" << endl;
    cout << "Laser Min Distance : " << laser->getMinRange() << " m\n"
         << endl;
}
void initializationSonarOne(Sonar *sonar_1) //sonar 1 initialization
{
    cout << "\nSonar Model : " << sonar_1->getModel() << endl;
    cout << "Sonar Field of view : " << sonar_1->getFieldOfView() << " degrees" << endl;
    cout << "Sonar Angular Resolution : " << sonar_1->getAngularResolution() << endl;
    cout << "Sensing Type: " << sonar_1->getSensingMethod() << endl;
    cout << "Number of Samples : " << sonar_1->getNumberSamples() << endl;
    cout << "Sonar Max Distance : " << sonar_1->getMaxRange() << " m" << endl;
    cout << "Sonar Min Distance : " << sonar_1->getMinRange() << " m\n"
         << endl;
}
void initializationSonarTwo(Sonar *sonar_2) //sonar 2 intialization
{
    cout << "\nSonar Model : " << sonar_2->getModel() << endl;
    cout << "Sonar Field of view : " << sonar_2->getFieldOfView() << " degrees" << endl;
    cout << "Sonar Angular Resolution : " << sonar_2->getAngularResolution() << " degrees" << endl;
    cout << "Sensing Type: " << sonar_2->getSensingMethod() << endl;
    cout << "Number of Samples : " << sonar_2->getNumberSamples() << endl;
    cout << "Sonar Max Distance : " << sonar_2->getMaxRange() << " m" << endl;
    cout << "Sonar Min Distance : " << sonar_2->getMinRange() << " m\n"
         << endl;
}
void configureLaser(Laser *laser) //configure laser Angular resolution and offset
{
    int user_input_config;
    double offset;
    cout << "\nConfiguring Laser" << endl;
    cout << "Laser Current Configuration # " << laser->getLaserConfiguration() << endl;
    cout << "Laser Current Angular Resolution : " << laser->getAngularResolution() << " degrees" << endl;
    cout << "Availible Number of Laser Configurations : " << laser->getNumberOfLaserConfigurations() << endl;
    cout << "Please Enter a Valid Configuration Number  to Change Angular Resolution" << endl;
    while (!(cin >> user_input_config))
    {
        cin.clear();
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        cout << "Invalid input, Try again: ";
    }
    switch (user_input_config) //switch for valid laser configurations
    {
    case 1:
        laser->setLaserConfiguration(1);
        cout << "Laser Configuration # " << laser->getLaserConfiguration() << endl;
        cout << "Laser  Angular Resolution : " << laser->getAngularResolution() << " degrees" << endl;
        break;
    case 2:
        laser->setLaserConfiguration(2);
        cout << "Laser Configuration # " << laser->getLaserConfiguration() << endl;
        cout << "Laser  Angular Resolution : " << laser->getAngularResolution() << " degrees" << endl;
        break;
    default:
        //if invalid default parameters used instead
        cout << "Invalid Parameters Used. Default Parameters used Instead" << endl;
        cout << "Laser Configuration # " << laser->getLaserConfiguration() << endl;
        cout << "Laser  Angular Resolution : " << laser->getAngularResolution() << " degrees" << endl;
        break;
    }
    cout << "Please Enter an Orientation Offset" << endl; // configures laser offset
    while (!(cin >> offset))
    {
        cin.clear();
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        cout << "Invalid input, Try again: ";
    }
    laser->setOffset(offset);
    cout << "Laser Offset : " << laser->getOffset() << " degrees\n"
         << endl;
}
void configureSonarOne(Sonar *sonar_1) //configures sonar 1 offser
{
    double offset;
    cout << "\nConfiguring Sonar 1" << endl;
    cout << "Please Enter an Orientation Offset" << endl;
    while (!(cin >> offset))
    {
        cin.clear();
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        cout << "Invalid input, Try again: ";
    }
    sonar_1->setOffset(offset);
    cout << "Sonar 1 Offset : " << sonar_1->getOffset() << " degrees\n"
         << endl;
}
void configureSonarTwo(Sonar *sonar_2) // configures sonar offset
{
    double offset;
    cout << "\nConfiguring Sonar 2" << endl;
    cout << "Please Enter an Orientation Offset" << endl;
    while (!(cin >> offset))
    {
        cin.clear();
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        cout << "Invalid input, Try again: ";
    }
    sonar_2->setOffset(offset);
    cout << "Sonar 2 Offset : " << sonar_2->getOffset() << " degrees\n"
         << endl;
}

void sensorsSettingsSummary(Laser *laser, Sonar *sonar_1, Sonar *sonar_2) //prints sensor summary configurations
{
    cout << "\n Summary of sensors Configuration :" << endl;
    cout << "\nLaser Model : " << laser->getModel() << endl;
    cout << "Laser Field of view : " << laser->getFieldOfView() << " Degrees" << endl;
    cout << "Laser Configuration # " << laser->getLaserConfiguration() << endl;
    cout << "Laser Angular Resolution : " << laser->getAngularResolution() << " Degrees" << endl;
    cout << "Sensing Type: " << laser->getSensingMethod() << endl;
    cout << "Number of Samples : " << laser->getNumberSamples() << endl;
    cout << "Laser Max Distance : " << laser->getMaxRange() << " m" << endl;
    cout << "Laser Min Distance : " << laser->getMinRange() << " m" << endl;
    cout << "Laser Offset : " << laser->getOffset() << " Degrees\n"
         << endl;
    cout << "----------------------------------------------------------------------------" << endl;
    cout << "\nSonar Model : " << sonar_1->getModel() << endl;
    cout << "Sonar Field of view : " << sonar_1->getFieldOfView() << " Degrees" << endl;
    cout << "Sonar Angular Resolution : " << sonar_1->getAngularResolution() << " Degrees" << endl;
    cout << "Sensing Type: " << sonar_1->getSensingMethod() << endl;
    cout << "Number of Samples : " << sonar_1->getNumberSamples() << endl;
    cout << "Sonar Max Distance : " << sonar_1->getMaxRange() << " m" << endl;
    cout << "Sonar Min Distance : " << sonar_1->getMinRange() << " m" << endl;
    cout << "Sonar 1 Offset : " << sonar_1->getOffset() << " Degrees\n"
         << endl;
    cout << "----------------------------------------------------------------------------" << endl;
    cout << "\nSonar Model : " << sonar_2->getModel() << endl;
    cout << "Sonar Field of view : " << sonar_2->getFieldOfView() << " Degrees" << endl;
    cout << "Sonar Angular Resolution : " << sonar_2->getAngularResolution() << " Degrees" << endl;
    cout << "Sensing Type: " << sonar_2->getSensingMethod() << endl;
    cout << "Number of Samples : " << sonar_2->getNumberSamples() << endl;
    cout << "Sonar Max Distance : " << sonar_2->getMaxRange() << " m" << endl;
    cout << "Sonar Min Distance : " << sonar_2->getMinRange() << " m" << endl;
    cout << "Sonar 1 Offset : " << sonar_2->getOffset() << " Degrees\n"
         << endl;
    cout << "----------------------------------------------------------------------------\n"
         << endl;
}

//Remaps cells locations
void cellsMapSize(vector<Cell *> &cells, Laser *laser, Sonar *sonar_1, Sonar *sonar_2)
{
    vector<double> getMaxRangeSensors{laser->getMaxRange(), sonar_1->getMaxRange(), sonar_2->getMaxRange()}; // store sensor max range
    double max = *std::max_element(getMaxRangeSensors.begin(), getMaxRangeSensors.end());                    // get sensor max range value
    int seed = std::chrono::system_clock::now().time_since_epoch().count();                                  //seed for data generation
    std::default_random_engine gen(seed);                                                                    //randomize
    for (int i = 0; i < cells.size(); i++)                                                                   //Loop vectors of pointers of cell
    {
        std::uniform_real_distribution<double> distX(-max, max); //get value for x position in plane
        std::uniform_real_distribution<double> distY(0.0, max);  //get value for y position in plane
        cells.at(i)->setCentre(distX(gen), distY(gen));          //set new centres for cells
        double x, y;                                             //values for new centres
        cells.at(i)->getCentre(x, y);                            // get new centre values
        cout << "cell # " << std::setw(2) << i + 1 << " (" << std::setw(9) << x << " ," << std::setw(9) << y << ") "
             << " side " << std::setw(3) << cells.at(i)->getSide() << " m " << endl;
    }
}
