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

	float orient[2]; // unit vector descirbing orientation
	Point centroid; // [x/y]
	Point center; // [x,y]
	Point projection; // [x/y]
	float projectionDistance;
	int area; // area behind this Frontier
	float reward; // reward for this Frontier
	float cost; // cost of travel to frontier for owning agent

	vector<Point> obstacles;
	vector<vector<Point> > obsClusters;

	vector<Point> members; // [list][x/y]
	bool editFlag;

	void getCentroid(Costmap &costmap);
	void getCenter();
	void getProjection();
	void getOrientation(Costmap &costMap);



};

#endif /* FRONTIER_H_ */