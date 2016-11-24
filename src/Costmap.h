/*
 * Costmap.h
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#ifndef COSTMAP_H_
#define COSTMAP_H_


#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

using namespace std;
using namespace cv;

class Costmap {
public:
	Costmap();
	virtual ~Costmap();

	void prettyPrintCostmap();
	//void findFrontiers();
	void shareCostmap(Costmap &b);

	// useful stuff
	int obsFree, frontier, unknown, obsWall;
	// 1 = free space // 2 = frontier
	// 101 = unknown
	// 201 = wall
	Vec3b cObsFree, cFrontier, cUnknown, cObsWall;

	Mat cells;
	Mat exploreReward;

	vector<Point> cellUpdates;
	vector<Point> viewPerim;

	void getExploreRewardMat(float frontierReward, float unknownReward);
	void displayExploreRewardHeatMat(Mat &mat);
	void simulateObservation(Point pose, Mat &resultingView);
	float getPoseReward(Mat &mat);
	bool visibleLineCheck(Point pose, Point target);

	float getEuclidianDistance( Point a, Point b);
	float aStarDist(Point sLoc, Point gLoc);
	vector<Point> aStarPath(Point sLoc, Point gLoc);


	Mat displayPlot;
	void buildCellsPlot(); // build nice display plot
	void showCostmapPlot(int index); // show nice display plot and number it
	void addAgentToPlot(Scalar color, vector<Point> myPath, Point cLoc);
};

#endif /* COSTMAP_H_ */
