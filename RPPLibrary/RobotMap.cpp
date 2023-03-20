#include "RobotMap.h"
#include <iostream>
#include <iomanip>
#include <queue>







namespace RPP
{
    
    Obstacle Obstacle::createObstacle(int x, int y, int radius) 
    {
        // Do some validation 
        if (radius <= 0) {
            throw std::invalid_argument("Radius of obstacle at (" + std::to_string(x) + "," + std::to_string(y) + ") must be greater than zero.");
        }

        // Create obstacle object
        Node obstacleCenterNode(x, y);
        Obstacle obstacle(obstacleCenterNode, radius);
        return obstacle;
    }
    
    Map::Map(int numRows, int numCols) : numRows_(numRows), numCols_(numCols) 
    {
        if (numRows <= 0 || numCols <= 0) {
            throw std::invalid_argument("Matrix size can not be negative or zero");
        }
        grid_.reserve(numRows);
        for (int row = 0; row < numRows_; ++row) {
            grid_.emplace_back();
            grid_.back().reserve(numCols);
            for (int col = 0; col < numCols_; ++col) {
                grid_.back().emplace_back(row, col);
            }
        }
        setNeighbors(); // set neighbors for each node
    }

    void Map::setNeighbors() {
        for (int row = 0; row < numRows_; ++row) {
            for (int col = 0; col < numCols_; ++col) {
                Node* node = &grid_[row][col];
                for (int r = -1; r <= 1; ++r) {
                    for (int c = -1; c <= 1; ++c) {
                        if (r == 0 && c == 0) {
                            continue; // Skip the current node
                        }
                        int nrow = row + r;
                        int ncol = col + c;
                        if (nrow >= 0 && nrow < numRows_ && ncol >= 0 && ncol < numCols_) {
                            node->getNeighbors().push_back(&grid_[nrow][ncol]);
                        }
                    }
                }
            }
        }
    }
  
    void Map::createObstacle(int x, int y, int radius)
    {
        bool partial = false;
        // Validate obstacles that are fully out of bounds
        for (int row = x - radius; row <= x + radius; ++row) {
            for (int col = y - radius; col <= y + radius; ++col) {
                if (row >= 0 && row < numRows_ && col >= 0 && col < numCols_) {
                    double dx = row - x;
                    double dy = col - y;
                    double distance = std::sqrt(dx * dx + dy * dy);
                    if (distance <= radius) {
                        // If we enter this if statement that means obstacle is at least partially bounds.
                        // Create obstacle and add it to the list
                        Obstacle obstacle = Obstacle::createObstacle(x, y, radius);
                        obstaclesList_.push_back(obstacle);
                        return;
                    }
                }
            }
        }

        // at this point not part of the obstacle is in the map so we cna throw an invalid argument.
        throw std::invalid_argument("Obstacle at position at (" + std::to_string(x) + "," + std::to_string(y) + ") will be fully outside the map boundaries");
        
    }

    void Map::addObstaclesToMap(const std::vector<Obstacle>& obstaclesList) {
        for (const Obstacle& obstacle : obstaclesList) {
            Node center = obstacle.getObstacleCenterNode();
            int radius = obstacle.getObstacleRadius();
            for (int row = center.getRow() - radius; row <= center.getRow() + radius; ++row) {
                for (int col = center.getCol() - radius; col <= center.getCol() + radius; ++col) {
                    if (row < 0 || col < 0 || row >= numRows_ || col >= numCols_) {
                        // Skip cells outside the map bounds
                        continue;
                    }
                    Node& node = grid_[row][col];
                    if (node.isObstacle()) {
                        // Skip cells already marked as obstacles
                        continue;
                    }
                    double dx = row - center.getRow();
                    double dy = col - center.getCol();
                    double distance = std::sqrt(dx * dx + dy * dy);
                    if (distance <= radius) {
                        node.setObstacle(true);
                        node.setDistance(0);
                        
                    }
                }
            }
        }

        // Run untill all nodes have values
        bool changed = true;
        while (changed) {
            changed = false;
            for (int row = 0; row < numRows_; ++row) {
                for (int col = 0; col < numCols_; ++col) {
                    Node* node = &grid_[row][col];
                    int minDistance = INT_MAX;
                    for (Node* neighbor : node->getNeighbors()) {
                        // print neighbors
                        //std::cout << "Neighbor row: " << neighbor->getRow() << " col: " << neighbor->getCol() << std::endl;
                        if (!neighbor->isObstacle() && neighbor->getDistance() != INT_MAX) {
                            int distance = neighbor->getDistance() + 1;
                            if (distance < minDistance && node->getDistance() > distance) {
                                minDistance = distance;
                                //std::cout << "1 Setting : Node row: " << node->getRow() << " col: " << node->getCol() << " set to " << minDistance << std::endl;
                            }
                        }
                        else if (neighbor->isObstacle() && node->getDistance() != 1 && node->getDistance() != 0) {
                           minDistance = 1;
                           //std::cout << "2 Setting : Node row: " << node->getRow() << " col: " << node->getCol() << " set to " << minDistance << std::endl;
                        }
                    }
                    if (minDistance != INT_MAX) {
                        //std::cout << "Node Distance was at " << node->getDistance() << std::endl;
                        //std::cout << "3 Setting : Node row: " << node->getRow() << " col: " << node->getCol() << " set to " << minDistance << std::endl;
                        
                        node->setDistance(minDistance);
                        changed = true;
                    }
                }
            }
        }
    }


    void Map::printToConsole(bool showBinary) const
    {
        if (showBinary) 
        {
            // Print the binary map
            std::cout << "Binary Map:" << std::endl;
            for (int row = 0; row < numRows_; ++row) {
                std::cout << std::setw(2) << row << "| ";
                for (int col = 0; col < numCols_; ++col) {
                    if (grid_[row][col].isObstacle()) {
                        std::cout << "\033[31m"; // red
                        std::cout << std::setw(6) << " 1  ";
                    }
                    else if (grid_[row][col].isStart()) {
                            std::cout << "\033[34m"; // blue
                            std::cout << std::setw(6) << " 0  ";
                    }
                    else if (grid_[row][col].isEnd()) {
                            std::cout << "\033[32m"; // green
                            std::cout << std::setw(6) << " 0  ";
                    }
                    else if (grid_[row][col].isRobot()) {
                        std::cout << "\033[33m"; // Yellow
                        std::cout << std::setw(6) << " 1  ";
                    }
                    else if (grid_[row][col].isBestPath()) {
                        std::cout << "\033[35m"; // Magenta
                        std::cout << std::setw(6) << " 0  ";
                    }
                    else if (grid_[row][col].isPath()) {
                        std::cout << "\033[36m"; // Cyan
                        std::cout << std::setw(6) << " 0  ";
                    }
                    else {
                            std::cout << "\033[0m";
                            std::cout << std::setw(6) << " 0  ";
                    }
                }
                std::cout << "\033[0m";
                std::cout << std::endl;
            }

            // Print the column numbers
            std::cout << "  +";
            for (int c = 0; c < numCols_; ++c) {
                std::cout << "   ---";
            }
            std::cout << std::endl << "  ";
            for (int c = 0; c < numCols_; ++c) {
                std::cout << std::setw(6) << c;
            }
            std::cout << std::endl;
        }
        else 
        {
            std::cout << "Distance Map:" << std::endl;
            for (int row = 0; row < numRows_; ++row) {
                std::cout << "\033[0m"; // white
                std::cout << std::setw(2) << row << "| ";
                for (int col = 0; col < numCols_; ++col) {
                    if (grid_[row][col].isObstacle()) {
                        std::cout << "\033[31m"; // red
                        
                    }
                    else if (grid_[row][col].isStart()) {
                        std::cout << "\033[34m"; // blue
                        
                    }
                    else if (grid_[row][col].isEnd()) {
                        std::cout << "\033[32m"; // green
                        
                    }
                    else if (grid_[row][col].isRobot()) {
                        std::cout << "\033[33m"; // Yellow
                        
                    }
                    else if (grid_[row][col].isBestPath()) {
                        std::cout << "\033[35m"; // Magenta
                    }
                    else if (grid_[row][col].isPath()) {
                        std::cout << "\033[36m"; // Cyan
                        
                    }
                    else {
                        std::cout << "\033[0m"; // white
                    }
                    if (grid_[row][col].getDistance() == INT_MAX) {
                        std::cout << std::setw(6) << "INF ";
                    }
                    else {
                        std::cout << std::setw(6) << grid_[row][col].getDistance();
                    }
                }
                std::cout << std::endl;
            }
            std::cout << "\033[0m"; // white
            
            // Print the column numbers
            std::cout << std::setw(3) << "-";
            for (int c = 0; c < numCols_; ++c) {
                std::cout << std::setw(6) << " ---";
            }
            std::cout << std::endl << "  ";
            for (int c = 0; c < numCols_; ++c) {
                std::cout << std::setw(6) << c;
            }
            std::cout << std::endl;
        }
    }



}