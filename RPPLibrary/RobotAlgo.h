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
			map_(*(Map*)nullptr),  // temporary Map object to bind to map_ reference
			startNode_(nullptr),  // temporary Node object to bind to startNode_ reference
			endNode_(nullptr),  // temporary Node object to bind to endNode_ reference
			robotPosition_(nullptr),
			path_(),
			algorithmType_(AlgorithmType::Dijkstra),
			robotRadius_(0),
			v_(false)
		{}

		Algorithm(Map& map, Node* startNode, Node* endNode, AlgorithmType algorithmType, int robotRadius);

		// Setters
		void setMap(Map& map) { map_ = map; }

		// Setters
		void setRobotPosition(Node* node, bool v);
		void setAlgorithmType(AlgorithmType algorithmType) { algorithmType_ = algorithmType; }
		void setRobotRadius(int robotRadius) { robotRadius_ = robotRadius; }
		
		// Getters
		Map* getMap() const { return &map_; }
		Node* getStartNode() const { return startNode_; }
		Node* getEndNode() const { return endNode_; } 
		Node* getRobotPosition() const { return robotPosition_; }
		std::vector<Node> getPath() const { return path_; }
		AlgorithmType getAlgorithmType() const { return algorithmType_; }
		int getRobotRadius() { return robotRadius_; }
		
		void printHeuristic();

		// Start the algorithm
		void startPathPlanning(bool v);

		
		
	private:
		Map& map_;
		Node* startNode_;
		Node* endNode_;
		Node* robotPosition_;
		std::vector<Node> path_;
		AlgorithmType algorithmType_;
		int robotRadius_;
		bool v_;
		
		void setNodeHeuristic();
		
		// Dynamic Visualizer
		void visualizer();

	};
}