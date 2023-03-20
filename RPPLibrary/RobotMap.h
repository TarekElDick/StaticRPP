// RPP.h
#pragma once
#include <string>
#include <memory>
#include <limits>
#include <vector>

namespace RPP
{
    class Node {
        
        double max = 999999999;

        public:
      
            Node(int x = -1, int y = -1)
                :
                x_(x),
                y_(y),
                isObstacle_(false),
                distanceToObstacle_(INT_MAX),
                isStart_(false),
                isEnd_(false),
                isRobot_(false),
                isPath_(false),
                isBestPath_(false),
                heuristic_(max),
                gScore_(0.0),
                f_(0.0),
                parent_(nullptr)
                
            {}

            // Setters
            void setObstacle(bool isObstacle) { isObstacle_ = isObstacle; }
            void setDistance(int distance) { distanceToObstacle_ = distance; }
            void setIsStart() { isStart_ = true; }
            void setIsEnd() { isEnd_ = true; }
            void setIsRobot(bool isRobot) { isRobot_ = isRobot; }
            void setIsPath(bool isPath) { isPath_ = isPath; }
            void setIsBestPath(bool isBestPath) { isBestPath_ = isBestPath; }
            void setNeighbors(std::vector<Node*>& neighbors) { neighbors_ = neighbors; }
            void setHeuristic(double heuristic) { heuristic_ = heuristic; }
            void setGScore(double gScore) { gScore_ = gScore; }
            void setF(double f) { f_ = f; }
            void setParent(Node* parent) { parent_ = parent; }

            // Getters
            int getRow() const { return x_; }
            int getCol() const { return y_; }
            bool isObstacle() const { return isObstacle_; }
            int getDistance() const { return distanceToObstacle_; }
            bool isStart() const { return isStart_; }
            bool isEnd() const { return isEnd_; }
            bool isRobot() const { return isRobot_; }
            bool isPath() const { return isPath_; }
            bool isBestPath() const { return isBestPath_; }
            std::vector<Node*>& getNeighbors() { return neighbors_; }
            double getHeuristic() { return heuristic_; }
            double getGScore() { return gScore_; }
            double getF() { return f_; }
            Node* getParent() { return parent_; }

        private:
        
            int x_;
            int y_;
            bool isObstacle_;
            int distanceToObstacle_;
            bool isStart_;
            bool isEnd_;
            bool isRobot_;
            bool isPath_;
            bool isBestPath_;
            std::vector<Node*> neighbors_;
            double heuristic_;
            double gScore_;
            double f_;
            Node* parent_;

    };

    class Obstacle {
    
        public:
        
            Obstacle()
                :
                obstacleCenterNode_(),
                obstacleRadius_(0)
            {}

            Obstacle(Node obstacleCenterNode, int radius) 
                : 
                obstacleCenterNode_(obstacleCenterNode), 
                obstacleRadius_(radius) 
            {}

            // Setters
            void setObstacleCenterNode(Node obstacleCenterNode) {obstacleCenterNode_ = obstacleCenterNode;}
            void setObstacleRadius(int obstacleRadius) { obstacleRadius_ = obstacleRadius; }

            // Getters
            Node getObstacleCenterNode() const { return obstacleCenterNode_; }
            int getObstacleRadius() const{ return obstacleRadius_; }

            // Static factory function
            static Obstacle createObstacle(int x, int y, int radius);

        private:
        
            Node obstacleCenterNode_;
            int obstacleRadius_;
    };

    class Map {
    
        public:
            
            // Default constructor
            Map() 
                : 
                numRows_(0),
                numCols_(0),
                grid_(),
                obstaclesList_()
            {}
            
            

            // Constructor with arguments
            Map(int numRows , int numCols);
            void setNeighbors();// set neighbors for each node
            
            // Getters
            int getNumRows() const { return numRows_; }
            int getNumCols() const { return numCols_; }
            std::vector<std::vector<Node>>& getGrid() { return grid_; }
            std::vector<Obstacle> getObstaclesList() const { return obstaclesList_; }

            // Member Functions
            void createObstacle(int x, int y, int radius);
            void addObstaclesToMap(const std::vector<Obstacle>& obstaclesList);
            void printToConsole(bool showBinary) const;      


        private:                                                                   
            int numRows_;                                                                               
            int numCols_;                                                                               
            std::vector<std::vector<Node>> grid_;                                                       
            std::vector<Obstacle> obstaclesList_;                                                       
        
        
        
    };

}




