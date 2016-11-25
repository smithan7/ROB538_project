/*
 * World.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */

#include "World.h"

using namespace cv;
using namespace std;

World::World(string fName, int resolution, float obsThresh, float comThresh){

	this->obsThresh = obsThresh;
	this->commThresh = comThresh;

	string fileName = fName + ".jpg";
	Mat image = imread(fileName,1);

	cvtColor(image,image,CV_BGR2GRAY);
	threshold(image,image,230,255,THRESH_BINARY);

	/*
	namedWindow("map in", WINDOW_NORMAL);
	imshow("map in", image);
	waitKey(10);
	*/

	initializeMaps(image, resolution);
	cout << "world::costmap.cells.size(): " << costmap.cells.cols << " x " << costmap.cells.rows << endl;
	cout << "World::Finished building " << fName << ".yml" << endl;
}

bool World::commoCheck(Point aLoc, Point bLoc, float comThresh){

	if( pow(aLoc.x-bLoc.x,2)+pow(aLoc.y-bLoc.y,2) > pow(comThresh,2)){
		return false;
	}

	Mat ta = Mat::zeros(costmap.cells.size(), CV_8UC1);
	LineIterator it(ta, aLoc, bLoc, 4, false);
	for(int i=0; i<it.count; i++, ++it){
		Point pp  = it.pos();
		//circle(ta, pp, 1, Scalar(255), -1, 8);
		if(pp.x >= costmap.cells.cols || pp.y >= costmap.cells.rows || costmap.cells.at<short>(pp) > costmap.obsFree){
			return false;
		}
		else if(costmap.cells.at<short>(pp) != costmap.obsFree){
			return false;
		}
	}
	return true;

}

void World::initializeMaps(Mat &imgGray, int resolution){

	costmap.cells = Mat::zeros(imgGray.rows / resolution, imgGray.cols / resolution, CV_16S);
	Mat temp = Mat::zeros(costmap.cells.size(), CV_8UC1);

	resize(imgGray, temp, temp.size(), 0, 0, INTER_AREA);

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			Scalar intensity = temp.at<uchar>(a);
			if(intensity[0] >= 225){
				costmap.cells.at<short>(a) = costmap.obsFree;
			}
			else{
				costmap.cells.at<short>(a) = costmap.obsWall;
			}
		}
	}
}


void World::observe(Point cLoc, Costmap &costmap){

	if(viewPerim.size() == 0){
		Mat temp =Mat::zeros(2*(obsThresh + 1), 2*(obsThresh + 1), CV_8UC1);
		Point cent(obsThresh,obsThresh);
		circle(temp,cent,obsThresh, Scalar(255));

		for(int i=0; i<temp.rows; i++){
			for(int j=0; j<temp.cols; j++){
				if(temp.at<uchar>(i,j,0) == 255){
					Point t(i-obsThresh, j-obsThresh);
					viewPerim.push_back(t);
				}
			}
		}
	}

	// if needed, initialize costmap
	if(costmap.cells.empty()){
		costmap.cells = Mat::ones(this->costmap.cells.size(), CV_16S) * costmap.unknown;
	}

	costmap.cellUpdates.clear();
	costmap.cellUpdates = getObservableCells(cLoc);

	// set obstacles in costmap
	float pOcc = 0.65;
	float pFree = 0.49;
	for(size_t i=0; i<costmap.cellUpdates.size(); i++){
		Point c = costmap.cellUpdates[i];
		if(costmap.cells.at<short>(c) != costmap.obsFree){
			costmap.cells.at<short>(c) = costmap.obsFree;
			for(int k=c.x-1; k<c.x+2; k++){
				for(int l=c.y-1; l<c.y+2; l++){
					Point a(k,l);
					if(this->costmap.cells.at<short>(a) == this->costmap.obsWall){ // are any of my nbrs visible?
						costmap.cells.at<short>(a) = costmap.obsWall; // set my cost
					}
				}
			}
		}
	}
}

vector<Point> World::getObservableCells(Point p){
	Mat ta = Mat::zeros(costmap.cells.size(), CV_8UC1);
	vector<Point> obsCellList;

	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + p.x, viewPerim[i].y + p.y);
		LineIterator it(ta, p, v, 4, false);
		for(int i=0; i<it.count; i++, ++it){
			Point pp  = it.pos();
			if(pp.x >= costmap.cells.cols || pp.y >= costmap.cells.rows || costmap.cells.at<short>(pp) > costmap.obsFree){
				break;
			}
			else if(costmap.cells.at<short>(pp) == costmap.obsFree){
				obsCellList.push_back(pp);
			}
		}
	}
	return obsCellList;
}

World::~World() {

}
