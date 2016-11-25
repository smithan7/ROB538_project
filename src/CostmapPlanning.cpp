/*
 * CostmapPlanning.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#include "CostmapPlanning.h"

bool pointCompare(Point &a, Point &b); // in costmap.cpp
bool pointOnMat(Point &a, Mat &b); // in costmap.cpp

CostmapPlanning::CostmapPlanning() {}

CostmapPlanning::~CostmapPlanning() {}

Point CostmapPlanning::greedyFrontierPlanner(Costmap &costmap, Point pLoc){

	vector< Point > oSet;
	oSet.push_back(pLoc);
	vector<float> oCost;
	oCost.push_back(0);
	float cCost = 0;

	vector< Point > cSet;

	while(oSet.size() > 0){

		// find lowest cost in oSet and set as pLoc
		int mindex;
		float mincost = INFINITY;
		for(size_t i=0; i<oSet.size(); i++){
			if(oCost[i] < mincost){
				mindex = i;
				mincost = oCost[i];
			}
		}

		pLoc = oSet[mindex];
		cCost = oCost[mindex];

		oSet.erase(oSet.begin() + mindex);
		oCost.erase(oCost.begin() + mindex);

		cSet.push_back(pLoc);

		// find nbrs of pLoc, if a frontier, return it; if not in cSet or oSet, add to oSet and calc cost
		int dx[4] = {1,-1,0,0};
		int dy[4] = {0,0,-1,1};
		for(int i=0; i<4; i++){
			Point tLoc(pLoc.x+dx[i], pLoc.y+dy[i]);
			float tCost = cCost + 1;

			if(costmap.cells.at<short>(tLoc) == costmap.frontier){ // frontier?
				return tLoc;
			}
			else if(costmap.cells.at<short>(tLoc) == costmap.obsFree){ // add to openset
				bool flag = true;
				for(size_t i=0; i<oSet.size(); i++){ // in oSet?
					if(pointCompare(oSet[i], tLoc) ){
						if(tCost < oCost[i]){
							oCost[i] = tCost;
						}
						flag = false;
						break;
					}
				}
				if(flag){
					for(size_t i=0; i<cSet.size(); i++){ // in cSet?
						if(cSet[i] == tLoc){
							flag = false;
							break;
						}
					}
				}
				if(flag){ // obsFree, not in oSet or cSet
					oSet.push_back(tLoc);
					oCost.push_back(tCost);
				}
			}
		}
	}
	return pLoc;
}

