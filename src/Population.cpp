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
	this->learningRate = 2;

	this->rewardStar = 0;
	this->constStar = constants[0];
	this->indexStar = 0;
}

void Population::getStar(){

	rewardStar = -INFINITY;
	for(int i=0; i<popSize; i++){
		if( rewards[i] > rewardStar){
			rewardStar = rewards[i];
			indexStar = i;
			constStar = constants[i];
		}
	}
}

void Population::mutate(){

	vector<int> maxList;

	/*
	cerr << endl << "rewards: " << endl;
	for(int i=0; i<popSize; i++){
		cerr << rewards[i] << endl;
	}
	*/

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
	vector<float> r;
	for(int i=0; i<popSize/2; i++){
		m.push_back( constants[maxList[i]] );
		r.push_back(rewards[maxList[i]] );
		vector<float> t;
		for( int j=0; j<nConst; j++){
			t.push_back(constants[maxList[i]][j] + learningRate * ( float(rand() % 100) / 100 - 0.5) );
		}
		m.push_back( t );
		r.push_back( -1 );
	}
	constants = m;
	rewards = r;
}

void Population::reset(){
	open.clear();
	rewards.clear();
	for(int i=0; i<popSize; i++){
		open.push_back(i);
		rewards.push_back(-INFINITY);
	}
}


Population::~Population() {
	// TODO Auto-generated destructor stub
}

