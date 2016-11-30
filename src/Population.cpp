/*
 * Population.cpp
 *
 *  Created on: Nov 29, 2016
 *      Author: andy
 */

#include "Population.h"

Population::Population(int popSize, int nConst) {

	for(int i=0; i<popSize; i++){
		vector<float> t;
		for(int j=0; j<nConst; j++){
			t.push_back(float(rand() % 1000) / 100 - 5); // rand -50 -> 50 // frontier cells, frontier travel cost, dComArea, dRealys, dExplorers, relay travel cost
		}
		constants.push_back( t );
		rewards.push_back(-INFINITY);
		open.push_back(i);
	}
	this->popSize = popSize;
	this->nConst = nConst;
}

void Population::mutate(){

	vector<int> maxList;

	for(int i = 0; i<popSize/2; i++){
		int maxI = -1;
		float maxV = -INFINITY;

		for(int j=0; j<popSize; j++){
			if( rewards[j] > maxV ){
				maxV = rewards[i];
				maxI = i;
			}
		}

		maxList.push_back(maxI);
		rewards[maxI] = -1;
	}
	vector<vector<float> > m;
	for(int i=0; i<popSize/2; i++){
		m.push_back( constants[maxList[i]] );
		vector<float> t;
		for( int j=0; j<nConst; j++){
			t.push_back(constants[maxList[i]][j] + float(rand() % 100 / 100) - 0.5);
		}
		m.push_back( t );
	}
	constants = m;
}

void Population::reset(){
	open.clear();
	for(int i=0; i<popSize; i++){
		open.push_back(i);
	}
}


Population::~Population() {
	// TODO Auto-generated destructor stub
}

