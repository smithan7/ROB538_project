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

	// for finding the optimal poses
	Graph thinGraph, poseGraph, travelGraph, tempGraph;

	float observedReward(Mat &observed, Mat &reward);
	void simulateObservation(Point pose, Mat &resultingView, Costmap &costmap);
	void findPosesEvolution(Costmap &costmap);
	void getOset(vector<int> &oStates, vector<int> &cStates, vector<int> &workingSet);

	void marketPoses( Costmap &costmap, Market &market );
	float getPoseReward( Point in, Costmap &costmap);
	void getViews(Costmap &costmap, vector<int> cPoses, Mat &cView, float &cReward, vector<Point> &poses, vector<Mat> &views, vector<float> poseRewards, float &gReward);
	void plotPoses( Costmap &costmap, vector<int> &cPoses, vector<int> &oPoses, vector<Mat> &views, vector<Point> &poses);
	float getDiscountedRewards( Costmap &costmap, vector<Point> &locs, int i, float globalReward );
	void simulateNULLObservation(Point pose, Mat &resultingView, Costmap &costmap);



	Point posePathPlanningTSP(Graph &graph, Costmap &costmap, vector<Point> &agentLocs, int &myIndex);
	void displayPoseTours(Costmap &costmap);
	void tspPoseFullTourPlanner(vector<Point> &agentLocs, int &myIndex);
	float getGraphObservations(Graph &graph, Costmap &costmap, Mat &gView, vector<int> &workingSet);

	void findPoseGraphTransitions(Graph &graph, Costmap &costmap);
	void displayPoseGraph(Costmap &costmap);
	void findPosesEvolution(Graph &graph, Costmap &costmap, vector<Point> &agentLocs);
	int matReward(Mat &in);
	vector<int> getWorkingSet(Graph &graph, Costmap &costmap);
	float getCurrentPoseSetReward(Graph &graph, Costmap &costmap, Mat &cView, vector<int> &workingSet);
	void getOset(Graph &graph, Costmap &costmap, vector<int> &oStates, vector<int> &cStates, vector<int> &workingSet);

	int nPullsTSP;
	int nPullsEvolvePoseGraph;
	int minObsForPose;
	vector<Point> viewPerim;

	vector<vector<int> > tspPoseTours; // [agent, pose along tour]
	vector<vector<Point > > tspPoseToursLocations;

};

#endif /* GRAPHCOORDINATION_H_ */
