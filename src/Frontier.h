/*
 * Frontier.h
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#ifndef FRONTIER_H_
#define FRONTIER_H_


#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

#include "Costmap.h"

using namespace std;

class Frontier {
public:
	Frontier(vector<Point> members);
	virtual ~Frontier();

	Point center; // [x,y]
	float reward; // reward for this Frontier
	float cost; // cost of travel to frontier for owning agent
	float value; // reward - cost

	bool editFlag; // for clustering

	vector<Point> members; // [list][x/y]

	void getCentroid(Costmap &costmap);
	void getCenter();



};

#endif /* FRONTIER_H_ */
