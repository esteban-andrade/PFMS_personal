#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <algorithm>
#include "laser.h"
#include "sonar.h"
#include "rangerfusion.h"
#include "cell.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;

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
    Laser laser;
    Sonar sonar_1;
    Sonar sonar_2;

    cout << "Starting Sensor" << endl;
    initializationLaser(&laser);
    initializationSonarOne(&sonar_1);
    initializationSonarTwo(&sonar_2);
    //  laser.generateData();
    //  sonar_1.generateData();

    //  vector<double> sonartest = sonar_1.getData();

    //  vector<double> test = laser.getData();

    // for (auto elem : laser.generateData())
    // {

    //     cout << "\nLaser " << elem << "\t" << endl;
    // }
    // for (auto elem : sonar_1.generateData())
    // {
    //     cout << "\nSonar " << elem << "\t" << endl;
    // }
    configureLaser(&laser);
    configureSonarOne(&sonar_1);
    configureSonarTwo(&sonar_2);
    sensorsSettingsSummary(&laser, &sonar_1, &sonar_2);

    vector<Cell *> cells;
    cells.clear();
    int user_cell_input;

    cout << "Please Enter the number of Cells that need to be created : " << endl;
    while (!(cin >> user_cell_input) || user_cell_input <= 0)
    {
        cin.clear();
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        cout << "Invalid input, Try again: ";
    }

    cells.reserve(user_cell_input + 1);

    for (int i; i < user_cell_input; i++)
    {
        cells.push_back(new Cell());
    }

    cout << "Number of cells created : " << cells.size() << endl;
    //cout << cells.capacity() << endl;
    cellsMapSize(cells, &laser, &sonar_1, &sonar_2);

    RangerFusion fusion;
    vector<RangerInterface *> rangers = {&laser, &sonar_1, &sonar_2};
    fusion.setRangers(rangers);

    fusion.setCells(cells);
    point_t origin;
    origin.x = 0;
    origin.y = 0;
    fusion.setOrigin(origin);

    // vector<vector<double>> raw_data = fusion.getRawRangeData();
    cout << "\n Starting Fusion : " << endl;
    int index = 0;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        fusion.grabAndFuseData();
        vector<vector<double>> raw_data = fusion.getRawRangeData();
        // for (int i = 0; i < raw_data.size(); i++)
        // {
        //     for (int j = 0; j < raw_data.at(i).size(); j++)
        //     {
        //         cout << "[" << raw_data.at(i).at(j) << "] ";
        //     }
        //     cout << endl;
        // }
        index++;
        cout << "Sample" << std::setw(4) << index << ":";
        for (int i = 0; i < cells.size(); i++)
        {
            cout << std::setw(4) << cells.at(i)->getState();
        }
        cout << endl;
    }
    return 0;
}

void initializationLaser(Laser *laser)
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
void initializationSonarOne(Sonar *sonar_1)
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
void initializationSonarTwo(Sonar *sonar_2)
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
void configureLaser(Laser *laser)
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
    switch (user_input_config)
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
        cout << "Invalid Parameters Used. Default Parameters used Instead" << endl;
        cout << "Laser Configuration # " << laser->getLaserConfiguration() << endl;
        cout << "Laser  Angular Resolution : " << laser->getAngularResolution() << " degrees" << endl;
        break;
    }
    /* cout << "Please Enter an Orientation Offset" << endl;
    while (!(cin >> offset))
    {
        cin.clear();
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        cout << "Invalid input, Try again: ";
    }
    laser->setOffset(offset);
    cout << "Laser Offset : " << laser->getOffset() << " degrees\n"
         << endl;*/
}
void configureSonarOne(Sonar *sonar_1)
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
void configureSonarTwo(Sonar *sonar_2)
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

void sensorsSettingsSummary(Laser *laser, Sonar *sonar_1, Sonar *sonar_2)
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

void cellsMapSize(vector<Cell *> &cells, Laser *laser, Sonar *sonar_1, Sonar *sonar_2)
{
    vector<double> getMaxRangeSensors{laser->getMaxRange(), sonar_1->getMaxRange(), sonar_2->getMaxRange()};
    double max = *std::max_element(getMaxRangeSensors.begin(), getMaxRangeSensors.end());
    for (int i = 0; i < cells.size(); i++)
    {
        // cout << "cell#" << i << " \t" << cells.at(i)->area() << endl;
        // cout << "cell#" << i << " \t" << cells.at(i)->getSide() << endl;
        // cout << "cell#" << i << " \t" << cells.at(i)->perimeter() << endl;
        int seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine gen(seed);
        std::uniform_real_distribution<double> distX(-max, max);
        std::uniform_real_distribution<double> distY(0.0, max);
        cells.at(i)->setCentre(distX(gen), distY(gen));
        double x, y;

        cells.at(i)->getCentre(x, y);
        cout << "cell # " << std::setw(2) << i + 1 << " (" << std::setw(9) << x << " ," << std::setw(9) << y << ") "
             << " side " << std::setw(3) << cells.at(i)->getSide() << " m " << endl;
    }
}
