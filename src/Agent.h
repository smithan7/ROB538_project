/*
 * Agent.h
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#ifndef AGENT_H_
#define AGENT_H_


#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include "World.h" // includes - costmap.h
#include "CostmapCoordination.h" // includes - world.h, frontier.h
#include "Market.h" // for graphCoordination
#include "Graph.h" // for planning over
#include "GraphCoordination.h"

using namespace std;

class Agent{
public:

	// agent stuff
	Agent(Point sLoc, int myIndex, World &gMap, float obsThresh, float comThresh, int numAgents, vector<float> constants);
	void pickMyColor();
	~Agent();
	void showCellsPlot();
	void communicate(Costmap &cIn, Market &mIn);
	int myIndex;
	Scalar myColor;
	float comThresh;
	float obsThresh;

	Point localPoseSearch();
	Point planExplore();
	vector<float> kRoleSwapping;
	Point planRelay();
	void planRoleSwapping();
	int role;
	void act();

	Market market;
	Graph poseGraph, comGraph;

	Point cLoc, gLoc, oLoc; // for map
	vector<Point> myPath;
	vector<Point> pathHistory;
	vector<char> roleHistory;

	// costmap class stuff
	Costmap costmap;
	CostmapCoordination costmapCoordination;
	GraphCoordination graphCoordination;

	int marketNodeSelect(World &gMap);
	void greedyFrontiers();
};

#endif /* AGENT_H_ */
