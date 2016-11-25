/*
 * Frontier.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#include "Frontier.h"

Frontier::Frontier(vector<Point> q) {

	center.x = -1;
	center.y = -1;

	reward = 1;
	cost = INFINITY;
	editFlag = false;
	members = q;
}

Frontier::~Frontier() {

}

void Frontier::getCenter(){
	float tx = 0;
	float ty = 0;
	for(size_t k=0; k<this->members.size(); k++){ // and get cumulative distance to all other members
		tx += members[k].x;
		ty += members[k].y;
	}

	center.x = round(tx / (float)this->members.size());
	center.y = round(ty / (float)this->members.size());

}

void Frontier::getCentroid(Costmap &costmap){
	float minDist = INFINITY;
	int minDex;
	for(size_t j=0; j<this->members.size(); j++){ // go through each cluster member
		float tempDist = 0;
		for(size_t k=0; k<this->members.size(); k++){ // and get cumulative distance to all other members
			tempDist += sqrt( pow(members[k].x - members[j].x,2) + pow(members[k].y - members[j].y,2));
		}
		if(tempDist < minDist){
			minDist = tempDist;
			minDex = j;
		}
	}
	this->center = members[minDex];
}
