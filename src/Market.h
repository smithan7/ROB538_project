/*
 * Market.h
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#ifndef MARKET_H_
#define MARKET_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

using namespace cv;
using namespace std;

class Market {
public:
	Market();
	void init(int nAgents, int myIndex);
	void shareMarket( Market &in );
	void updateMarket( Point cLoc, Point gLoc );
	void iterateTime();
	void printMarket();

	vector<float> nnGetAgentVals();
	vector<float> nnGetObserverVals();

	virtual ~Market();

	vector<Point> cLocs;
	vector<Point> gLocs;
	vector<float> costs;
	vector<float> values;
	vector<float> rewards;
	vector<char> roles;

	vector<int> times;
	bool contactWithObserver;

	int myIndex;
	int nAgents;

};

#endif /* MARKET_H_ */
