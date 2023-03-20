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
        //      Initalize map object with the desired number of Rows, Columns.
        RPP::Map myMap(100,20);

        // 2) Create your ostacles this function adds the ostacles in a list. 
        myMap.createObstacle(5, 5, 1);
        myMap.createObstacle(17, 5, 3);
        myMap.createObstacle(25, 7, 2);
        myMap.createObstacle(50, 10, 4);
        myMap.addObstaclesToMap(myMap.getObstaclesList());

        //myMap.printToConsole(1);
        
        RPP::Node startNode(1,1);
        RPP::Node endNode(99, 19);
        RPP::AlgorithmType algoType = RPP::AlgorithmType::AStar;


        RPP::Algorithm myAlgo(myMap, &startNode, &endNode, algoType, 1);

        myAlgo.startPathPlanning(false);
        
        myMap.printToConsole(0);
        myMap.printToConsole(1);
        myAlgo.printHeuristic();

        
    }
    catch (const std::invalid_argument& e) {
        std::cout << "Exception caught: " << e.what() << std::endl;
    }

    std::cout << "Finished ";
    return 0;

}