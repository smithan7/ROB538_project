/*
 * GraphCoordination.h
 *
 *  Created on: Nov 7, 2016
 *      Author: andy
 */

#ifndef GRAPHCOORDINATION_H_
#define GRAPHCOORDINATION_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


#include "Frontier.h" // frontiers
#include "Graph.h"
#include "Market.h"

using namespace cv;
using namespace std;


class GraphCoordination {
public:
	GraphCoordination();
	virtual ~GraphCoordination();

	void init(int obsRadius, int comRadius, vector<float> constants);
	vector<float> constants;

	// for finding the optimal poses
	Graph thinGraph;

	Point relayPlanning( Costmap &costmap, Market &market, Point oLoc, Point &rLoc );
	int getCoveredRelays( Costmap &costmap, Market &market, Mat &comMat, Point oLoc );
	int getCoveredRelays(Market &market, Mat &comMat );
	int getCoveredExplorers( Mat &comMat, Market &market );
	void simulateCommunication(Point pose, Mat &comMat, Costmap &costmap);
	bool commoCheck(Point aLoc, Point bLoc, Costmap &costmap);
	float comRadius;
	float rReward;
	vector<Point> viewPerim, comPerim;

};

#endif /* GRAPHCOORDINATION_H_ */
