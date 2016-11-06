/*
 * Market.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#include "Market.h"

Market::Market(){}

void Market::init(int nAgents, int myIndex) {
	for(int i=0; i<nAgents; i++){
		Point a(0,0);
		cLocs.push_back(a);
		time.push_back(0);
	}
	this->myIndex = myIndex;
}

Market::~Market() {}

void Market::shareMarket( Market &vis ){

	for( size_t i=0; i<this->cLocs.size(); i++){
		if(this->time[i] >= vis.time[i]){
			this->cLocs[i] = vis.cLocs[i];
			this->time[i] = vis.time[i];
		}
	}
}

void Market::printMarket(){
	cout << "cLocs: ";
	for( size_t i=0; i<this->cLocs.size(); i++){
		cout << cLocs[i];
		if(i+1 < cLocs.size() ){
			cout << ", ";
		}
	}
	cout << endl;

	cout << "time: ";
	for( size_t i=0; i<this->time.size(); i++){
		cout << time[i];
		if(i+1 < time.size() ){
			cout << ", ";
		}
	}
	cout << endl;
}

void Market::updateMarket( Point cLoc ){
	for( size_t i=0; i<this->cLocs.size(); i++){
		time[i]++;
	}
	cLocs[myIndex] = cLoc;
	time[myIndex] = 0;
}

void Market::iterateTime(){
	for( size_t i=0; i<this->cLocs.size(); i++){
		time[i]++;
	}
}
