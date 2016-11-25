/*
 * Observer.h
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "Costmap.h"
#include "Market.h"

class Observer {
public:
	Observer(Point cLoc, int nAgents, bool global, String name);
	virtual ~Observer();
	void communicate(Costmap &cIn, Market &mIn);

	bool globalObserver;
	String name;

	Point cLoc;
	int nAgents;
	vector<Scalar> agentColors;

	Costmap costmap;
	Market market;

	void showCellsPlot();
	void addAgentsToCostmapPlot();
	Scalar setAgentColor(int i);
	void addSelfToCostmapPlot();
};


#endif /* OBSERVER_H_ */
