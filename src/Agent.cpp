/*
 * Agent.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

using namespace std;

#include "Agent.h"

Agent::Agent(Point sLoc, int myIndex, World &gMap, float obsThresh, float comThresh, int numAgents){
	this->obsThresh = obsThresh;
	this->comThresh = comThresh;
	//this->myMap.createGraph(gMap, obsThresh, comThresh, gMap.gSpace);

	cLoc= sLoc;
	gLoc = sLoc;

	this->myIndex = myIndex;
	this->pickMyColor();

	market.init(numAgents, myIndex );
}

void Agent::communicate(Costmap &cIn, Market &mIn){
	this->costmap.shareCostmap(cIn);
	this->market.shareMarket( mIn );
}

void Agent::act(){

	if(cLoc == gLoc){
		while(true){
			Point g;
			g.x = gLoc.x + rand() % 5 - 2;
			g.y = gLoc.y + rand() % 5 - 2;

			if(costmap.cells.at<short>(g) == costmap.obsFree){
				gLoc = g;
				break;
			}
		}
	}

	myPath = costmap.aStarPath(cLoc, gLoc);
	cLoc = myPath[1];
	myPath.erase(myPath.begin());
	//cout << "Agent::act::cLoc / gLoc: " << cLoc << " / " << gLoc << endl;
	//cout << "Agent::act::out" << endl;

	history.push_back(cLoc);
	market.updateMarket(cLoc);
}


void Agent::soloPlan(string method, int timeLeft){

	if(method.compare("greedy") == 0){
		if(costmap.cells.at<short>(gLoc) != costmap.infFree || (gLoc.x == cLoc.x && gLoc.y == cLoc.y) ){
			cout << "going into costmapPlanning.GreedyFrontierPlanner" << endl;
			gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
			cout << "gLoc: " << gLoc.x << " , " << gLoc.y << endl;
		}
	}
	else if(method.compare("select") == 0){

		float w[3] = {1, 1, 0.5}; // explore, search, map
		float e[2] = {0.5, 5}; // dominated, breaches
		float spread = 0.5; // spread rate
		costmap.getRewardMat(w, e, spread);
		costmap.displayThermalMat( costmap.reward );

		//gLoc = costmapPlanning.explorePlanner(costmap, cLoc);
		//gLoc = costmapPlanning.searchPlanner(costmap, cLoc);
		gLoc = costmapPlanning.mappingPlanner(costmap, cLoc);

		if(gLoc.x < 0 || gLoc.y < 0){
			cerr << "greedy" << endl;
			if(costmap.cells.at<short>(gLoc) != costmap.infFree || (gLoc.x == cLoc.x && gLoc.y == cLoc.y) ){
				gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
			}
		}
	}
	//cout << "out of solo plan" << endl;
}


void Agent::showCellsPlot(){
	costmap.buildCellsPlot();

	circle(costmap.displayPlot,cLoc,2, myColor,-1, 8);
	circle(costmap.displayPlot,gLoc,2, Scalar(0,0,255),-1, 8);

	char buffer[50];
	sprintf(buffer,"Agent[%d]::costMat", myIndex);

	namedWindow(buffer, WINDOW_NORMAL);
	imshow(buffer, costmap.displayPlot);
	waitKey(1);
}

void Agent::pickMyColor(){
	this->myColor = {0,0,0};

	if(this->myIndex == 0){
		this->myColor[0] = 255;
	}
	else if(this->myIndex == 1){
		this->myColor[1] = 255;
	}
	else if(this->myIndex == 2){
		this->myColor[2] = 255;
	}
	else if(this->myIndex == 3){
		this->myColor[0] = 255;
		this->myColor[1] = 153;
		this->myColor[2] = 51;
	}
	else if(this->myIndex == 4){
		this->myColor[0] = 255;
		this->myColor[1] = 255;
		this->myColor[2] = 51;
	}
	else if(this->myIndex == 5){
		this->myColor[0] = 255;
		this->myColor[1] = 51;
		this->myColor[2] = 255;
	}
	else if(this->myIndex == 6){
		this->myColor[0] = 51;
		this->myColor[1] = 255;
		this->myColor[2] = 255;
	}
	else if(this->myIndex == 7){
		this->myColor[0] = 153;
		this->myColor[1] = 255;
		this->myColor[2] = 51;
	}
	else if(this->myIndex == 8){
		this->myColor[0] = 255;
		this->myColor[1] = 255;
		this->myColor[2] = 255;
	}
	else if(this->myIndex == 9){
		// white
	}
}

Agent::~Agent() {

}

