/*
 * CostmapPlanning.h
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#ifndef COSTMAPPLANNING_H_
#define COSTMAPPLANNING_H_


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

#include "Costmap.h"
#include "Contour.h"

using namespace std;

class CostmapPlanning {
public:
	CostmapPlanning();
	virtual ~CostmapPlanning();

	Point greedyFrontierPlanner(Costmap &costmap, Point cLoc);
};

#endif /* COSTMAPPLANNING_H_ */
