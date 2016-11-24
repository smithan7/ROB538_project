/*
 * CostmapCoordination.h
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#ifndef COSTMAPCOORDINATION_H_
#define COSTMAPCOORDINATION_H_


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

#include "World.h" // includes - costmap.h
#include "Frontier.h"
#include "Market.h"

using namespace std;

class CostmapCoordination {
public:
	CostmapCoordination();
	virtual ~CostmapCoordination();
	void init( float obsThresh, vector<float> constants );

	float obsThresh;

	// Frontiers
	vector<Frontier> frontiers; // graphs frontiers of item
	vector<Point> findFrontiers(Costmap &costmap); // search Graph and find Frontiers
	void clusterFrontiers(vector<Point> frntList, Costmap &costmap); // cluster Frontiers into groups

	// market frontiers
	Point marketFrontierPlanner(Costmap &costmap, Market &market);
	void marketFrontiers( Costmap &costmap, Market &market);
	vector<float> eConstants;
	float eReward;

	// useful functions
	void findClosestFrontier(Costmap &costmap, Point cLoc, int &goalIndex, float &goalDist);
	void plotFrontiers(Costmap &costmap, vector<Point> &frontierCells);

};

#endif /* COSTMAPCOORDINATION_H_ */
