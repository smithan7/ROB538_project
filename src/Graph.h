/*
 * Graph.h
 *
 *  Created on: Nov 7, 2016
 *      Author: andy
 */

#ifndef GRAPH_H_
#define GRAPH_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

#include "Costmap.h" // includes - n/a

using namespace cv;
using namespace std;

class Graph {
public:
	Graph();
	virtual ~Graph();

	// build graph
	int nodeSpacing;
	int nbrSpacing;

	int cNode; // my location
	vector<Point> nodeLocations;
	vector<bool> nullView;
	vector<vector<float> > nodeTransitions;
	vector<Mat> nodeObservations;
	vector<int> nodeSubGraphIndices;

	Mat nullObservationNodes;

	vector<int> cPath;

	void clearTransitions( int i );

	void createThinGraph(Costmap &costmap, int nodeSpacing, int nbrSpacing);
	void updateThinGraph(Costmap &costmap, int nodeSpacing, int nbrSpacing);
	Mat thinMat;
	Mat priorMat;
	void pruneThinMat( Costmap &costmap );
	void downSample( Mat &oMat, Mat &nMat, Costmap &costmap);

	void createPRMGraph(Point cLoc, Costmap &costmap, int nodeSpacing, int nbrSpacing);
	void thinning(const Mat& src, Mat& dst);
	void thinningIteration(Mat& img, int iter);
	void findStatesByGrid(Mat &inMat);
	void findStatesCityBlockDistance(Mat &thinMat);
	void findCNodeTransitions(Costmap &costmap);
	void findStateTransitionsCityBlockDistance(Costmap &costmap);
	void findStateTransitionsByVisibility(Costmap &costmap);
	bool checkVisibility(Costmap &costmap, float dist, Point a, Point b);
	void mergeStatesBySharedNbr();

	// useful things
	float aStarDist(int sIndex, int gIndex, Costmap &costmap);
	int findNearestNode(Point in, Costmap &costmap);
	void displayCoordMap(Costmap &costmap, bool displayNodes);
	bool visibleLineCheck(Costmap &costmap, Point pose, Point pt);
};

#endif /* GRAPH_H_ */
