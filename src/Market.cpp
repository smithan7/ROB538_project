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
		gLocs.push_back(a);
		times.push_back(0);
		costs.push_back(0);
		roles.push_back('e');
	}
	this->myIndex = myIndex;
	contactWithObserver = false;
	this->nAgents = nAgents;
}

Market::~Market() {}

void Market::shareMarket( Market &vis ){
	for( size_t i=0; i<this->cLocs.size(); i++){ // for all agents
		if(this->times[i] <= vis.times[i] && i != vis.myIndex){ // have i seen them sooner than they have?
			// yes, give them all my info on them
			vis.cLocs[i] = this->cLocs[i];
			vis.gLocs[i] = this->gLocs[i];
			vis.cLocs[i] = this->cLocs[i];
			vis.costs[i] = this->costs[i];
			vis.times[i] = this->times[i];

			if(this->contactWithObserver){
				vis.contactWithObserver = true;
			}
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
	for( size_t i=0; i<this->times.size(); i++){
		cout << time[i];
		if(i+1 < times.size() ){
			cout << ", ";
		}
	}
	cout << endl;
}

void Market::updateMarket( Point cLoc, Point gLoc ){
	this->iterateTime();
	this->contactWithObserver = false;

	cLocs[myIndex] = cLoc;
	gLocs[myIndex] = gLoc;
	times[myIndex] = 0;
}

void Market::iterateTime(){
	for( size_t i=0; i<this->cLocs.size(); i++){
		times[i]++;
	}
}

/*
vector<float> nnGetAgentVals(){
	vector<float> frntVals;
	for(int i=0; i<4; i++){
		frntVals.push_back(0);
	}

	for(size_t i=0; i<cells.cols; i++){
		for(size_t j=0; j<cells.rows; j++){
			Point t(i,j);
			if(cells.at<short>(t) == infFree){
				float distSq = distP2PSq(t, cLoc);

				int quad = getQuad( cLoc, t );
				frntVals[quad] += 1 / distSq;
			}
		}
	}

	return agentVals;
}
vector<float> nnGetObserverVals(){

	vector<float> frntVals;
	for(int i=0; i<4; i++){
		frntVals.push_back(0);
	}

	for(size_t i=0; i<cells.cols; i++){

	}
	return observerVals;
}
*/
