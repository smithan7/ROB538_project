/*
 * CostmapCoordination.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#include "CostmapCoordination.h"

CostmapCoordination::CostmapCoordination(){}

void CostmapCoordination::init(float obsThresh, vector<float> constants){
	this->obsThresh = obsThresh;
	this->eConstants.push_back( constants[0] );
	this->eConstants.push_back( constants[1] );
	this->eReward = 0;
}

CostmapCoordination::~CostmapCoordination() {}

Point CostmapCoordination::marketFrontierPlanner(Costmap &costmap, Market &market){

	// find frontiers and cluster them into cells
	frontiers.clear();
	vector<Point> frontierCells = findFrontiers(costmap);
	if(frontierCells.size() > 0){
		clusterFrontiers(frontierCells, costmap);
		marketFrontiers( costmap, market);
		//plotFrontiers(costmap, frontierCells);

		return market.gLocs[market.myIndex];
	}
	else{
		return Point(-1,-1);
	}
}

void CostmapCoordination::marketFrontiers(Costmap &costmap, Market &market){

	for(size_t i=0; i<frontiers.size(); i++){
		frontiers[i].getCenter();
	}

	// am I closer to any poses in market than the current purchaser?
	for(size_t i=0; i<market.gLocs.size(); i++){
		if( int(i) != market.myIndex ){
			bool flag = false;
			if( market.gLocs[i].x > 0 && market.gLocs[i].y > 0){
				float ed = costmap.getEuclidianDistance( market.cLocs[market.myIndex], market.gLocs[i]);
				if(ed  < market.costs[i] ){ // I am euclidian closer, check path distance
					float d = costmap.aStarDist(market.gLocs[i], market.cLocs[market.myIndex]);
					if(d > market.costs[i]-2){ // I am not a* closer, clear reward
						if( d < market.costs[i] && market.myIndex < int(i) ){

						}
						else{
							flag = true;
						}
					}
					else if( d == market.costs[i] && market.myIndex > int(i) ){
						flag = true; // I am equal distance but they out rank me :(
					}
				}
				else{ // I am not euclidian closer, clear reward
					flag = true;
				}
			}
			if(flag){ // they are closer, remove all reward from their goals
				for(size_t j=0; j<frontiers.size(); j++){
					float ed = costmap.getEuclidianDistance(frontiers[j].center, market.gLocs[i]);
					if( ed < 0.5*obsThresh ){ // clear frontiers close to their goal
						frontiers[j].reward = -1;
					}
				}
			}
		}
	}

	vector<bool> trueDist;
	int minI = -1;
	float minCost = INFINITY;
	bool rewardAvailable = false;

	for(size_t i=0; i<frontiers.size(); i++){
		if( frontiers[i].reward > 0){
			rewardAvailable = true;
		}
		trueDist.push_back( false );
		frontiers[i].cost = costmap.getEuclidianDistance( market.cLocs[market.myIndex], frontiers[i].center);
	}

	if( !rewardAvailable ){
		market.gLocs[market.myIndex] = Point(-1,-1);
		market.costs[market.myIndex] = 0;
		return;
	}
	// go through remaining frontiers, with rewards, and pick best one and get cost
	while( true ){ // find best pose

		minCost = INFINITY;
		minI = -1;
		for(size_t i=0; i<frontiers.size(); i++){
			if(frontiers[i].reward > 0 && frontiers[i].cost < minCost){
				minI = i;
				minCost = frontiers[i].cost;
			}
		}

		if( trueDist[minI]){
			break;
		}

		frontiers[minI].cost = costmap.aStarDist(market.cLocs[market.myIndex], frontiers[minI].center);
		trueDist[minI] = true;
	}

	// publish gLoc to the market and return it
	market.gLocs[market.myIndex] = frontiers[minI].center;
	market.costs[market.myIndex] = frontiers[minI].cost;

	this->eReward = eConstants[0] * float(frontiers[minI].members.size()) - eConstants[1]*frontiers[minI].cost;
}

void CostmapCoordination::plotFrontiers(Costmap &costmap, vector<Point> &frontierCells){

	Mat displayPlot= Mat::zeros(costmap.cells.size(), CV_8UC3);
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if(costmap.cells.at<short>(a) == costmap.obsFree){
				displayPlot.at<Vec3b>(a) = costmap.cObsFree;
			}
			else if(costmap.cells.at<short>(a) == costmap.obsWall){
				displayPlot.at<Vec3b>(a) = costmap.cObsWall;
			}
			else{
				displayPlot.at<Vec3b>(a) = costmap.cUnknown;
			}
		}
	}

	Vec3b red;
	red[0] = 127;
	red[1] = 127;
	red[2] = 255;
	for(size_t i=0; i<frontierCells.size(); i++){
		displayPlot.at<Vec3b>(frontierCells[i]) = red;
	}

	for(size_t i=0; i<frontiers.size(); i++){
		frontiers[i].getCenter();
		circle(displayPlot, frontiers[i].center, 2, Scalar(0,0,255), -1, 8);
		//  char text[2];
		//sprintf(text,"%d", i);
		//putText(displayPlot, text, frontiers[i].center, 0, 0.3, Scalar(0,0,255) );

	}

	namedWindow("frontiers", WINDOW_NORMAL);
	imshow("frontiers", displayPlot);
	waitKey(1);
}

vector<Point> CostmapCoordination::findFrontiers(Costmap &costmap){

	int dx[4] = {-1,1,0,0};
	int dy[4] = {0,0,-1,1};

	vector<Point> frontiersList;
	for(int i=1; i<costmap.cells.cols-1; i++){
		for(int j=1; j<costmap.cells.rows-1; j++){
			Point a(i,j);
			if(costmap.cells.at<short>(a) == costmap.obsFree){
				bool flag =false;
				for(int k=0; k<4; k++){
					Point b(a.x+dx[k], a.y+dy[k]);
					if(costmap.cells.at<short>(b) == costmap.unknown){
						flag = true;
					}
				}
				if(flag){
					frontiersList.push_back(a);
				}
			}
		}
	}
	return frontiersList;
}

void CostmapCoordination::clusterFrontiers(vector<Point >  frntList, Costmap &costmap){
	// check to see if frnt.center is still a Frontier cell, if so keep, else delete
	for(size_t i=0; i<frontiers.size(); i++){
		frontiers[i].editFlag = true;
		bool flag = true;
		for(size_t j=0; j<frntList.size(); j++){
			if(frontiers[i].center == frntList[j]){
				flag = false;
				frntList.erase(frntList.begin()+j);
			}
		}
		if(flag){
			frontiers.erase(frontiers.begin()+i);
		}
		else{
			frontiers[i].editFlag = false;
		}
	}
	// breadth first search through known clusters
	for(size_t i=0; i<frontiers.size(); i++){ // keep checking for new Frontier clusters while there are unclaimed Frontiers
		vector<Point> q; // current cluster
		vector<Point> qP; // open set in cluster
		qP.push_back(frontiers[i].center);

		while((int)qP.size() > 0){ // find all nbrs of those in q
			Point seed = qP[0];
			q.push_back(qP[0]);
			qP.erase(qP.begin(),qP.begin()+1);
			for(int ni = seed.x-2; ni<seed.x+3; ni++){
				for(int nj = seed.y-2; nj<seed.y+3; nj++){
					for(size_t i=0; i<frntList.size(); i++){
						if(frntList[i].x == ni && frntList[i].y == nj){
							qP.push_back(frntList[i]); // in range, add to open set
							frntList.erase(frntList.begin() + i);
						}
					}
				}
			}
		}
		this->frontiers[i].members = q; // save to list of clusters
	}

	// breadth first search
	while(frntList.size() > 0){ // keep checking for new Frontier clusters while there are unclaimed Frontiers
		vector<Point> q; // current cluster
		vector<Point> qP; // open set in cluster
		qP.push_back(frntList[0]);
		frntList.erase(frntList.begin());

		while((int)qP.size() > 0){ // find all nbrs of those in q
			Point seed = qP[0];
			q.push_back(qP[0]);
			qP.erase(qP.begin(),qP.begin()+1);
			for(int ni = seed.x-1; ni<seed.x+2; ni++){
				for(int nj = seed.y-1; nj<seed.y+2; nj++){
					for(int i=0; i<(int)frntList.size(); i++){
						if(frntList[i].x == ni && frntList[i].y == nj){
							qP.push_back(frntList[i]); // in range, add to open set
							frntList.erase(frntList.begin() + i, frntList.begin()+i+1);
						}
					}
				}
			}
		}
		Frontier a(q);
		this->frontiers.push_back(a);
	}
	for(size_t i=0; i<this->frontiers.size(); i++){ // number of clusters
		if(this->frontiers[i].editFlag){
			frontiers[i].getCenter();
		}
	}
}

void CostmapCoordination::findClosestFrontier(Costmap &costmap, Point cLoc, int &goalIndex, float &goalDist){
	float minDist = INFINITY;
	int mindex = -1;

	vector<float> dists;

	for(size_t i=0; i<frontiers.size(); i++){
		float dist = sqrt( pow(cLoc.x - frontiers[i].center.x,2) + pow(cLoc.y - frontiers[i].center.y,2));
		dists.push_back( dist );

		if(dist < minDist){
			minDist = dist;
			mindex = i;
		}
	}

	vector<bool> aDist(this->frontiers.size(), false);

	while(true){
		// get A* dist of closest
		if(!aDist[mindex]){
			dists[mindex] = costmap.aStarDist(cLoc, frontiers[mindex].center);
			aDist[mindex] = true;
		}
		else{
			break;
		}
		// is it still the closest with A* dist?
		for(size_t i=0; i<dists.size(); i++){
			if(dists[i] < minDist){
				minDist = dists[i];
				mindex = i;
			}
		}
	}

	goalIndex = mindex;
	goalDist = dists[mindex];

}

