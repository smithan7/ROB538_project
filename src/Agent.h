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
#include "CostmapPlanning.h" // includes - world.h, frontier.h
#include "Market.h" // for graphCoordination

using namespace std;

class Agent{
public:

	// agent stuff
	Agent(Point sLoc, int myIndex, World &gMap, float obsThresh, float comThresh, int numAgents);
	void pickMyColor();
	~Agent();
	void showCellsPlot();
	void communicate(Costmap &cIn, Market &mIn);
	int myIndex;
	Scalar myColor;
	float comThresh;
	float obsThresh;

	// working
	void soloPlan(string method, int timeLeft);
	void coordinatedPlan(string method, int timeLeft, vector<Agent> &agents);

	void act();

	Market market;

	Point cLoc, gLoc; // for map
	vector<Point> myPath;
	vector<Point> history;

	// costmap class stuff
	Costmap costmap;
	CostmapCoordination costmapCoordination;
	CostmapPlanning costmapPlanning;

	int marketNodeSelect(World &gMap);
	void greedyFrontiers();
};

#endif /* AGENT_H_ */
