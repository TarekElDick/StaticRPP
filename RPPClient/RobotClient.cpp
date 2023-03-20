#include <iostream>
#include <RobotMap.h>
#include <RobotAlgo.h>
#include "RobotClient.h"

using namespace std;

int main()
{
    bool redo = true;
    while (redo) {
        try {

            int width, height, startX, startY, endX, endY;
            cout << "Enter the width of the map: ";
            cin >> width;
            cout << "Enter the height of the map: ";
            cin >> height;
            RPP::Map myMap(width, height);
            cout << "Enter the number of obstacles you want to create: ";
            int numObstacles;
            cin >> numObstacles;
            for (int i = 0; i < numObstacles; i++) {
                int x, y, size;
                cout << "Enter the coordinates and size of obstacle " << i + 1 << ": ";
                cin >> x >> y >> size;
                myMap.createObstacle(x, y, size);
            }
            myMap.addObstaclesToMap(myMap.getObstaclesList());

            cout << "Enter the starting node coordinates: ";
            cin >> startX >> startY;
            RPP::Node startNode(startX, startY);
            cout << "Enter the ending node coordinates: ";
            cin >> endX >> endY;
            RPP::Node endNode(endX, endY);
            
            int robotRadius;
            cout << "Enter the radius of the robot: ";
            cin >> robotRadius;
            
            RPP::Algorithm myAlgo(myMap, &startNode, &endNode, robotRadius);

            bool visualize;
            cout << "Do you want to visualize the robots movement? (1 for Yes, 0 for No): ";
            cin >> visualize;
            if (visualize == 1 || visualize == 0) {
                myAlgo.startPathPlanning(visualize);
            }
            else {
                throw std::invalid_argument("Invalid entry");
            }

            cout << "Do you want to redo the test? (1 for Yes, 0 for No): ";
            cin >> redo;
        }
        catch (const std::invalid_argument& e) {
            std::cout << "Exception caught: " << e.what() << std::endl;
            redo = true;
        }
    }

    std::cout << "Finished ";
    return 0;
}
