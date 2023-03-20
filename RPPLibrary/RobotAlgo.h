#pragma once
#include "RobotMap.h"


namespace RPP
{
	enum class AlgorithmType {
		Dijkstra,
		AStar
	};
	
	class Algorithm
	{
	public:

		// Default contructor
		Algorithm()
			:
			map_(*(Map*)nullptr),  
			startNode_(nullptr), 
			endNode_(nullptr),
			robotPosition_(nullptr),
			path_(),
			robotRadius_(0),
			v_(false)
		{}

		// Creates an algorithm object
		Algorithm(Map& map, Node* startNode, Node* endNode, int robotRadius);

		// Setters
		void setMap(Map& map) { map_ = map; }

		// Setters
		void setRobotPosition(Node* node, bool v);
		void setRobotRadius(int robotRadius) { robotRadius_ = robotRadius; }
		
		// Getters
		Map* getMap() const { return &map_; }
		Node* getStartNode() const { return startNode_; }
		Node* getEndNode() const { return endNode_; } 
		Node* getRobotPosition() const { return robotPosition_; }
		std::vector<Node> getPath() const { return path_; }
		int getRobotRadius() const { return robotRadius_; }
		
		void printHeuristic();
		void startPathPlanning(bool v);

	private:
		Map& map_;
		Node* startNode_;
		Node* endNode_;
		Node* robotPosition_;
		std::vector<Node> path_;
		int robotRadius_;
		bool v_;
		
		void setNodeHeuristic();
		void visualizer();

	};
}