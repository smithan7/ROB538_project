/*
 * CostmapCoordination.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#include "CostmapCoordination.h"

CostmapCoordination::CostmapCoordination(){

}

void CostmapCoordination::initializeMarket(int nAgents){
	for(int i=0; i<nAgents; i++){
		standingBids.push_back(-1);
		Point t(-1,-1);
		goalLocations.push_back(t);
	}
}

CostmapCoordination::~CostmapCoordination() {}

void CostmapCoordination::plotFrontiers(Costmap &costmap, vector<Point> &frontierCells){

	Mat displayPlot= Mat::zeros(costmap.cells.size(), CV_8UC3);
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			if(costmap.cells.at<uchar>(i,j) == costmap.obsFree){
				displayPlot.at<Vec3b>(i,j) = costmap.cObsFree;
			}
			else if(costmap.cells.at<uchar>(i,j) == costmap.infFree){
				displayPlot.at<Vec3b>(i,j) = costmap.cInfFree;
			}
			else if(costmap.cells.at<uchar>(i,j) == costmap.obsWall){
				displayPlot.at<Vec3b>(i,j) = costmap.cObsWall;
			}
			else if(costmap.cells.at<uchar>(i,j) == costmap.infWall){
				displayPlot.at<Vec3b>(i,j) = costmap.cInfWall;
			}
			else if(costmap.cells.at<uchar>(i,j) == costmap.inflatedWall){
				displayPlot.at<Vec3b>(i,j) = costmap.cInfWall;
			}
			else if(costmap.cells.at<uchar>(i,j) == costmap.unknown){
				displayPlot.at<Vec3b>(i,j) = costmap.cUnknown;
			}
			else{ // anything else, should never happen
				displayPlot.at<Vec3b>(i,j) = costmap.cError;
			}
		}
	}

	Vec3b red;
	red[0] = 0;
	red[1] = 0;
	red[2] = 255;
	for(size_t i=0; i<frontierCells.size(); i++){
		displayPlot.at<Vec3b>(frontierCells[i]) = red;
	}


	for(size_t i=0; i<frontiers.size(); i++){
		circle(displayPlot, frontiers[i].centroid, 1, Scalar(0,0,255), -1, 8);

		char text[2];
		sprintf(text,"%d", i);
		putText(displayPlot, text, frontiers[i].centroid, 0, 0.3, Scalar(0,0,255) );

	}

	namedWindow("frontiers", WINDOW_NORMAL);
	imshow("frontiers", displayPlot);
	waitKey(1);
}

void CostmapCoordination::findClosestFrontier(Costmap &costmap, Point cLoc, int &goalIndex, float &goalDist){
	float minDist = INFINITY;
	int mindex = -1;

	vector<float> dists;

	for(size_t i=0; i<frontiers.size(); i++){
		float dist = costmap.getEuclidianDistance(cLoc, frontiers[i].centroid);
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
			dists[mindex] = costmap.aStarDist(cLoc, frontiers[mindex].centroid);
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

void CostmapCoordination::placeMyOrder(Costmap &costmap, Point cLoc, int myIndex){


	float minDist = INFINITY;
	int mindex = -1;

	vector<float> dists;

	for(size_t i=0; i<frontiers.size(); i++){
		float dist = costmap.getEuclidianDistance(cLoc, frontiers[i].centroid);
		dists.push_back( dist );

		if(dist < minDist){
			minDist = dist;
			mindex = i;
		}
	}

	vector<bool> aDist(this->frontiers.size(), false); // have I done A* to this frontier?

	while(true){
		// get A* dist of closest
		if(!aDist[mindex]){
			dists[mindex] = costmap.aStarDist(cLoc, frontiers[mindex].centroid);
			aDist[mindex] = true;
		}
		else{  // if the closest has an A* dist then stop
			break;
		}

		minDist = INFINITY;
		// find closest again
		for(size_t i=0; i<dists.size(); i++){ // for distances to all frontiers
			if(dists[i] <= minDist){ // is it the current closest? if not I don't care
				bool flag = false;
				for(size_t j=0; j<goalLocations.size(); j++){ // only care about those I can outbid
					if(j != myIndex && goalLocations[j].x > 0 && goalLocations[j].y > 0){ // don't compete against self and they have a goal
						if(costmap.getEuclidianDistance(frontiers[i].centroid, goalLocations[j]) < 5){ // is this frontier bid on?
							flag = true; // say it was bid on
							if(standingBids[j] < dists[i]){ // am I outbid?
								dists[i] = INFINITY; // yes, never use this one again
							}
							else if(standingBids[j] == dists[i] && myIndex >= j){ // am I tied and they outrank me?
								dists[i] = INFINITY; // yes, never use this one again
							}
							else{ // I am not outbid
								minDist = dists[i];
								mindex = i;
							}
						}
					}
				}
				if(!flag){ // it wasn't bid on
					minDist = dists[i];
					mindex = i;
				}
			}
		}
	}
	standingBids[myIndex] = dists[mindex];
	goalLocations[myIndex] = frontiers[mindex].centroid;
}

Point CostmapCoordination::marketFrontiers(Costmap &costmap, Point cLoc, int myIndex){

	// find frontiers and cluster them into cells
	vector<Point> frontierCells = findFrontiers(costmap);
	clusterFrontiers(frontierCells, costmap);

	placeMyOrder(costmap, cLoc, myIndex);

	//plotFrontiers(costmap, frontierCells);

	return goalLocations[myIndex];
}

vector<Point> CostmapCoordination::findFrontiers(Costmap &costmap){
	vector<Point> frontiersList;
	for(int i=1; i<costmap.cells.cols-1; i++){
		for(int j=1; j<costmap.cells.rows-1; j++){
			bool newFrnt = false;
			if(costmap.cells.at<uchar>(i,j) == costmap.infFree){ // i'm unobserved
				if(costmap.cells.at<uchar>(i+1,j) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<uchar>(i-1,j) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<uchar>(i,j+1) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<uchar>(i,j-1) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
			}
			if(newFrnt){
				Point fT(i,j);
				frontiersList.push_back(fT);
			}
		}
	}
	return frontiersList;
}

void CostmapCoordination::clusterFrontiers(vector<Point >  frntList, Costmap &costmap){
	// check to see if frnt.centroid is still a Frontier cell, if so keep, else delete
	for(size_t i=0; i<frontiers.size(); i++){
		frontiers[i].editFlag = true;
		bool flag = true;
		for(size_t j=0; j<frntList.size(); j++){
			if(frontiers[i].centroid == frntList[j]){
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
		qP.push_back(frontiers[i].centroid);

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
			//frontiers[i].getCentroid(costmap);
			frontiers[i].getCenter();
		}
	}
}

