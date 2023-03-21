#include "RobotAlgo.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <thread>


namespace RPP
{
	Algorithm::Algorithm(Map& map, Node* startNode, Node* endNode, int robotRadius ) 
		:
		map_(map),
		startNode_(nullptr),  // temporary Node object to bind to startNode_ reference
		endNode_(nullptr),  // temporary Node object to bind to endNode_ reference
		robotPosition_(nullptr),
		path_(),
		robotRadius_(robotRadius)
	{
		
		// Validate if start and end nodes are within bounds of the map
		if (startNode->getRow() < 0 || startNode->getRow() >= map.getNumRows() ||
			startNode->getCol() < 0 || startNode->getCol() >= map.getNumCols()) {
			throw std::invalid_argument("Start node is outside the bounds of the map.");
		}

		if (endNode->getRow() < 0 || endNode->getRow() >= map.getNumRows() ||
			endNode->getCol() < 0 || endNode->getCol() >= map.getNumCols()) {
			throw std::invalid_argument("End node is outside the bounds of the map.");
		}
		
		
		// TODO Validate that the starting node and end node are not touching obstacle nodes, and that the robot fits at the nodes.


		// Use a reference to the grid
		std::vector<std::vector<Node>>& grid = map.getGrid(); 

		// Get a reference to the start node in the grid and set it as the start node
		startNode_ = &grid[startNode->getRow()][startNode->getCol()];
		startNode_->setIsStart();

		// Get a reference to the end node in the grid and set it as the end node
		endNode_ = &grid[endNode->getRow()][endNode->getCol()];
		endNode_->setIsEnd();

		// Set the robots position on the map before we start algo.
		setRobotPosition(startNode_,0);

		// Set Heuristic of each Node
		setNodeHeuristic();
		
	}

	void Algorithm::setRobotPosition(Node * newRobotPosition, bool v) {
		
		// Use a reference to the grid
		std::vector<std::vector<Node>>& grid = map_.getGrid();
		
		// Remove old robot position if robotPosition_ not null.  
		if (robotPosition_ != nullptr) {
			for (int row = robotPosition_->getRow() - robotRadius_; row <= robotPosition_->getRow() + robotRadius_; ++row) {
				for (int col = robotPosition_->getCol() - robotRadius_; col <= robotPosition_->getCol() + robotRadius_; ++col) {
					if (row < 0 || col < 0 || row >= map_.getNumRows() || col >= map_.getNumCols()) {
						// Skip cells outside the map bounds
						continue;
					}
					Node* node = &grid[row][col];
					double dx = row - robotPosition_->getRow();
					double dy = col - robotPosition_->getCol();
					double distance = std::sqrt(dx * dx + dy * dy);
					if (distance <= robotRadius_) {
						if (node->isObstacle()) {
							throw std::invalid_argument("Robot position touching an obstacle at (" + std::to_string(row) + "," + std::to_string(col) + ")");
						}
						node->setIsRobot(false);
						
					}
				}
			}
		}
		
		//  Set the robots new position on the map.
		for (int row = newRobotPosition->getRow() - robotRadius_; row <= newRobotPosition->getRow() + robotRadius_; ++row) {
			for (int col = newRobotPosition->getCol() - robotRadius_; col <= newRobotPosition->getCol() + robotRadius_; ++col) {
				if (row < 0 || col < 0 || row >= map_.getNumRows() || col >= map_.getNumCols()) {
					// Skip cells outside the map bounds
					continue;
				}
				Node* node = &grid[row][col];
				double dx = row - newRobotPosition->getRow();
				double dy = col - newRobotPosition->getCol();
				double distance = std::sqrt(dx * dx + dy * dy);
				if (distance <= robotRadius_) {
					if (node->isObstacle()) {
						throw std::invalid_argument("Robot position touching an obstacle at (" + std::to_string(row) + "," + std::to_string(col) + ")");
					}
					node->setIsRobot(true);
					node->setIsPath(true);
				}
			}
		}
		robotPosition_ = newRobotPosition;
		if (v) {
			visualizer();
		}
	}
	
	void Algorithm::setNodeHeuristic() {
		// Use a reference to the grid
		std::vector<std::vector<Node>>& grid = map_.getGrid();
		double max = 999999999;
		// Run untill all nodes have values, exact Euclidean Distance heuristic. 
		bool changed = true;
		while (changed) {
			changed = false;
			for (int row = 0; row < map_.getNumRows(); ++row) {
				for (int col = 0; col < map_.getNumCols(); ++col) {
					Node* node = &grid[row][col];
					double minDistance = max;
					for (Node* neighbor : node->getNeighbors()) {
						if (node->isObstacle()) {
							// Skip obstacle cells
							//std::cout << "Here 1" << std::endl;
							continue;
						}
						if ((!node->isEnd() && neighbor->getHeuristic() != max) || neighbor->isEnd() == true) {

							if (neighbor->isEnd() == true) {
								neighbor->setHeuristic(0);
							}
							
							//std::cout << "Here 3" << std::endl;
							double dx = node->getCol() - neighbor->getCol();
							double dy = node->getRow() - neighbor->getRow();
							
							double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
							double distanceFromEnd = neighbor->getHeuristic() + distance;
							
							if (distanceFromEnd < minDistance && node->getHeuristic() > distanceFromEnd) {
								//std::cout << "Node: " << node->getRow() << ", " << node->getCol() << std::endl;
								//std::cout << "distanceFromEnd " << distanceFromEnd << " minDistance " << minDistance << " node->getHeuristic() " << node->getHeuristic() << std::endl;
								minDistance = distanceFromEnd;
							}
						} 
					}
					if (minDistance != max) {
						node->setHeuristic(minDistance);
						changed = true;
					}
				}
			}
		}
	}

	void Algorithm::printHeuristic() {
		
		std::vector<std::vector<Node>>& grid = map_.getGrid();

		std::cout << "Heuristic Map:" << std::endl;
		for (int row = 0; row < map_.getNumRows(); ++row) {
			std::cout << "\033[0m"; // white
			std::cout << std::setw(2) << row << "| ";
			for (int col = 0; col < map_.getNumCols(); ++col) {
				Node* node = &grid[row][col];
				if (node->isObstacle()) {
					std::cout << "\033[31m"; // red
				}
				else if (node->isStart()) {
					std::cout << "\033[34m"; // blue

				}
				else if (node->isEnd()) {
					std::cout << "\033[32m"; // green

				}
				else if (node->isRobot()) {
					std::cout << "\033[33m"; // Yellow

				}
				else if (node->isBestPath()) {
					std::cout << "\033[35m"; // Magenta
					
				}
				else if (node->isPath()) {
					std::cout << "\033[36m"; // Cyan

				}
				else {
					std::cout << "\033[0m"; // white
				}
				if (node->getHeuristic() == 999999999) {
					std::cout << std::setw(9) << "INF";
				}
				else {
					std::cout << std::setw(9) << node->getHeuristic();
				}
			}
			std::cout << std::endl;
		}
		std::cout << "\033[0m"; // white

		// Print the column numbers
		std::cout << std::setw(3) << "+";
		for (int c = 0; c < map_.getNumCols(); ++c) {
			std::cout << std::setw(9) << " ---";
		}
		std::cout << std::endl << "  ";
		for (int c = 0; c < map_.getNumCols(); ++c) {
			std::cout << std::setw(9) << c;
		}
		std::cout << std::endl;
	}

	void Algorithm::startPathPlanning(bool v) {
		
		// A* Search Algorithm

		// 1. Create an empty set of open nodes and a set of closed nodes
		std::vector<Node*> openSet, closedSet;

		// 2. Initialize the closed list put the starting node on the openlist(you can leave its f at zero)
		startNode_->setHeuristic(0);
		startNode_->setGScore(0);
		startNode_->setF(0);

		
		openSet.push_back(startNode_);

		// 3. While open list is not empty
		while (!openSet.empty()) {

			// a) Find the node with the least F on the open list 
			Node* current = openSet.front();
			int minIndex = 0;
			for (size_t i = 1; i < openSet.size(); i++) {
				if (openSet[i]->getF() < current->getF()) {
					current = openSet[i];
					minIndex = i;
				}
			}

			// Move to robot
			setRobotPosition(current, v);

			// b) Remove the current node from the open set
			openSet.erase(openSet.begin() + minIndex);

			// c) Add the current node to the closed set
			closedSet.push_back(current);

			// d) If we have reached the goal node, reconstruct the path.
			if (current->isEnd()) {
				
				while (!current->isStart()) {
					current->setIsBestPath(true);
					path_.push_back(*current);
					current = current->getParent();
				}
				path_.push_back(*startNode_);
				std::reverse(path_.begin(), path_.end());
				visualizer();
				return;
			}

			// e) Generate the neighbors of the current node
			std::vector<Node*> neighbors = current->getNeighbors();
			
			// f) Loop through the neighbors of the current node.
			for (Node* neighbor : neighbors) {
				
				// Validate which neighbors we can travel to.
				// If the neighbor obstacle or is already in the closed set, skip it
				if (neighbor->isObstacle() || find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()) {
					continue;
				}
				
				// Validate with the robot radius. 
				if (robotRadius_ >= neighbor->getDistance()) {
					continue;
				}

				// Validate that the robot doesnt go out of bounds while searching
				if ( (neighbor->getRow() + getRobotRadius()) > map_.getNumRows()-1 || (neighbor->getCol() + getRobotRadius()) > map_.getNumCols()-1 ) {
					continue;
				}
				// Validate that the robot doesnt go out of bounds while searching
				if ( (neighbor->getRow() - getRobotRadius()) < 0 || (neighbor->getCol() - getRobotRadius()) < 0) {
					continue;
				}



				// Calculate the tentative g score for the neighbor
				// calculate the difference of the movement.
				double dx = current->getCol() - neighbor->getCol();
				double dy = current->getRow() - neighbor->getRow();

				double gScore = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

				double tentativeGScore = current->getGScore() + gScore;

				// If the neighbor is not in the open set, add it
				if (find(openSet.begin(), openSet.end(), neighbor) == openSet.end()) {
					
					openSet.push_back(neighbor);
					neighbor->setGScore(tentativeGScore);
					neighbor->setF(neighbor->getGScore() + neighbor->getHeuristic());
					neighbor->setParent(current);
				}
				else {
				// If the neighbor is already in the open set, update its g score if the new score is lower
					if (tentativeGScore < neighbor->getGScore()) {

						neighbor->setGScore(tentativeGScore);
						neighbor->setF(neighbor->getGScore() + neighbor->getHeuristic());
						neighbor->setParent(current);
					}
				}
			}
				
		}

		// If we reach this point, it means there is no path from the start node to the goal node
		std::cout << "No Path Found!" << std::endl;
		
	}

	void Algorithm::visualizer() {

		#ifdef _WIN32
			system("cls");
		#else
			// Assume POSIX compliant
			system("clear");
		#endif
		map_.printToConsole(1); // print the current state of the map
		std::cout.flush(); // flush the output to ensure it is immediately displayed
		std::this_thread::sleep_for(std::chrono::milliseconds(200)); // delay for half a second
	
	}

}