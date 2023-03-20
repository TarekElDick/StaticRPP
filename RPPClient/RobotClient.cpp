#include <iostream>
#include <RobotMap.h>
#include <RobotAlgo.h>
#include "RobotClient.h"
using namespace std;


int main()
{

    // 1) Create Map Object 
    //      Initalize map object with the desired number of Rows, Columns.
    try {


        // 1) Create Map Object 
        // Initalize map object with the desired number of Rows, Columns.
        RPP::Map myMap(5,5);

        // 2) Create your ostacles this function adds the ostacles in a list. 
        myMap.createObstacle(2, 2, 1);
        myMap.addObstaclesToMap(myMap.getObstaclesList());

        string filename = "test1.bin";
        
        RPP::Map yourMap;

        myMap.saveToFile(filename);
        yourMap.loadFromFile(filename);
        
        RPP::Node startNode(0,0);
        RPP::Node endNode(4, 4);
        
        RPP::Algorithm myAlgo(yourMap, &startNode, &endNode, 0);

        myAlgo.startPathPlanning(true);

       
    }
    catch (const std::invalid_argument& e) {
        std::cout << "Exception caught: " << e.what() << std::endl;
    }

    std::cout << "Finished ";
    return 0;

}