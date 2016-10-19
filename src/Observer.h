/*
 * Observer.h
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "Costmap.h"
#include "Agent.h"

class Observer {
public:
	Observer(Point cLoc);
	virtual ~Observer();

	Point cLoc;

	Costmap costmap;
	void showCellsPlot(vector<Agent> agents);
	void addAgentsToCostmapPlot(vector<Agent> agents, Mat &displayPlot);

	vector<vector<int> > agentLocList;

};


#endif /* OBSERVER_H_ */
