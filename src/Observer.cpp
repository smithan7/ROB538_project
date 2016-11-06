/*
 * Observer.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */


#include "Observer.h"

Observer::Observer(Point cLoc, int nAgents, bool global, String name){
	this->cLoc = cLoc;
	this->nAgents = nAgents;
	globalObserver = global;
	this->name = name;

	for(int i=0; i<nAgents; i++){
		agentColors.push_back( setAgentColor(i) );
	}
	market.init(nAgents, -1);
}

Observer::~Observer(){}

void Observer::communicate(Costmap &cIn, Market &mIn){
	this->costmap.shareCostmap(cIn);
	this->market.shareMarket( mIn );
}

Scalar Observer::setAgentColor(int index){
	Scalar color(0,0,0);

	if(index == 0){
		color[0] = 255;
	}
	else if(index == 1){
		color[1] = 255;
	}
	else if(index == 2){
		color[2] = 255;
	}
	else if(index == 3){
		color[0] = 255;
		color[1] = 153;
		color[2] = 51;
	}
	else if(index == 4){
		color[0] = 255;
		color[1] = 255;
		color[2] = 51;
	}
	else if(index == 5){
		color[0] = 255;
		color[1] = 51;
		color[2] = 255;
	}
	else if(index == 6){
		color[0] = 51;
		color[1] = 255;
		color[2] = 255;
	}
	else if(index == 7){
		color[0] = 153;
		color[1] = 255;
		color[2] = 51;
	}
	else if(index == 8){
		color[0] = 255;
		color[1] = 255;
		color[2] = 255;
	}
	else if(index == 9){
		// white
	}

	return color;
}

void Observer::showCellsPlot(){
	costmap.buildCellsPlot();
	addAgentsToCostmapPlot();
	addSelfToCostmapPlot();
	namedWindow(name, WINDOW_NORMAL);
	imshow(name, costmap.displayPlot);
	waitKey(1);
}

void Observer::addAgentsToCostmapPlot(){
	for(int i=0; i<nAgents; i++){
		circle(costmap.displayPlot,market.cLocs[i],2, agentColors[i],-1, 8);
	}
}

void Observer::addSelfToCostmapPlot(){
	rectangle( costmap.displayPlot, Point(cLoc.x-2, cLoc.y-2), Point(cLoc.x+2, cLoc.y+2), Scalar(0,165,255), -1, 8);
}
