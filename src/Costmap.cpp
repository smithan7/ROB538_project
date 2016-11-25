/*
 * Costmap.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: andy
 */


#include "Costmap.h"

bool pointCompare(Point &a, Point &b);
bool pointOnMat(Point &a, Mat &b);
float distP2P( Point a, Point b);
float distP2PSq( Point a, Point b);
int getQuad( Point cLoc, Point t);

Costmap::Costmap(){
	this->cObsFree = {255,255,255};
	this->cFrontier = {200,200,200};
	this->cUnknown = {127,127,127};
	this->cObsWall = {0,0,0};

	this->obsFree = 1;
	this->frontier = 2;
	this->unknown = 101;

	this->obsWall = 201;

	int obsRadius = 17;
	Mat temp =Mat::zeros(2*(obsRadius + 1), 2*(obsRadius + 1), CV_8UC1);
	Point cent;
	cent.x = obsRadius;
	cent.y = obsRadius;
	circle(temp,cent,obsRadius, Scalar(255));

	for(int i=0; i<temp.cols; i++){
		for(int j=0; j<temp.rows; j++){
			if(temp.at<uchar>(i,j,0) == 255){
				Point t(i-obsRadius, j-obsRadius);
				viewPerim.push_back(t);
			}
		}
	}
}

void Costmap::shareCostmap( Costmap &cIn ){
	for(int i=0; i<this->cells.cols; i++){
		for(int j=0; j<this->cells.rows; j++){
			Point a(i,j);
			// share cells
			if(this->cells.at<short>(a) != cIn.cells.at<short>(a) ){ // do we think the same thing?
				if(cIn.cells.at<short>(a) == cIn.unknown){
					cIn.cells.at<short>(a) = this->cells.at<short>(a); // if this->doesn't know, anything is better
				}
				else if(cIn.cells.at<short>(a) == cIn.frontier){ // this->think its inferred
					if(this->cells.at<short>(a) == this->obsFree || this->cells.at<short>(a) == this->obsWall){ // B has observed
						cIn.cells.at<short>(a) = this->cells.at<short>(a);
					}
				}
			}
		}
	}
}

Costmap::~Costmap() {}

float distP2P( Point a, Point b){
	return sqrt( pow(a.x-b.x,2) + pow(a.y-b.y,2) );
}

float distP2PSq( Point a, Point b){
	return pow(a.x-b.x,2) + pow(a.y-b.y,2);
}


void Costmap::prettyPrintCostmap(){
	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point a(i,j);
			cout << cells.at<short>(a) << ",";
		}
		cout << endl;
	}
}

bool pointCompare(Point &a, Point &b){
	if(a.x == b.x && a.y == b.y){
		return true;
	}
	else{
		return false;
	}
}

bool pointOnMat(Point &a, Mat &b){
	if(a.x >= 0 && a.x < b.cols && a.y >= 0 && a.y < b.rows){
		return true;
	}
	else{
		return false;
	}
}

float Costmap::getEuclidianDistance( Point a, Point b){
	return sqrt( pow(a.x-b.x,2) + pow(a.y-b.y,2) );
}

void Costmap::getExploreRewardMat( float frontierReward, float unknownReward ){
	// explore, search, map; dominated, breach; spread rate

	if(exploreReward.empty()){
		exploreReward = Mat::zeros(cells.size(), CV_32FC1);
	}

	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point a(i,j);
			// explore reward
			if(cells.at<short>(a) == frontier ){
				exploreReward.at<float>(a) = frontierReward;
			}
			else if(cells.at<short>(a) == unknown){
				exploreReward.at<float>(a) = unknownReward;
			}
			else{
				exploreReward.at<float>(a) = 0;
			}
		}
	}
}

void Costmap::displayExploreRewardHeatMat(Mat &mat){
	Mat tMat = Mat::zeros( cells.size(), CV_8UC3);

	float maxV = -1;

	for(int i=0; i<mat.cols; i++){
		for(int j=0; j<mat.rows; j++){
			Point a(i,j);
			if(mat.at<float>(a) > maxV){
				if( mat.at<float>(a) > 1 ){
					mat.at<float>(a) = 1;
					maxV = 1;
				}
				else{
					maxV = mat.at<float>(a);
				}
			}
		}
	}

	for(int i=0; i<mat.cols; i++){
		for(int j=0; j<mat.rows; j++){
			Point a(i,j);
			float r = mat.at<float>(a) / maxV;

			Vec3b color(0,0,0);
			if(r < 0.125){
				color[1] = 255;
				color[2] = 0;
			}

			else if( r > 0.875 ){
				color[1] = 0;
				color[2] = 255;
			}
			else{
				color[1] = round( 255 + (r+0.125)*-255/maxV );
				color[2] = round( (r-0.125)*255/(maxV-0.125) );
			}
			tMat.at<Vec3b>(a) = color;
		}
	}

	namedWindow("Thermal Mat", WINDOW_NORMAL);
	imshow("Thermal Mat", tMat);
	waitKey(1);
}

void Costmap::simulateObservation(Point pose, Mat &resultingView){
	// make perimeter of viewing circle fit on image

	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + pose.x, viewPerim[i].y + pose.y);
		LineIterator it(resultingView, pose, v, 8, false);
		for(int i=0; i<it.count; i++, ++it){

			Point pp  = it.pos();

			if(cells.at<short>(pp) == obsWall){
				break;
			}
			else{
				resultingView.at<uchar>(pp) = 255;
			}
		}
	}

	/*
	Mat fu = resultingView.clone();
	circle(fu, pose, 2, Scalar(127), -1, 8);
	namedWindow("GraphCoordination::simulateView::view", WINDOW_NORMAL);
	imshow("GraphCoordination::simulateView::view", fu);
	waitKey(0);
	*/
}

bool Costmap::visibleLineCheck(Point pose, Point target){
	// make perimeter of viewing circle fit on image

	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + pose.x, viewPerim[i].y + pose.y);
		LineIterator it(cells, pose, v, 8, false);
		for(int i=0; i<it.count; i++, ++it){

			Point pp  = it.pos();

			if(cells.at<short>(pp) > obsFree){
				return false;
			}
		}
	}
	return true;

	/*
	Mat fu = resultingView.clone();
	circle(fu, pose, 2, Scalar(127), -1, 8);
	namedWindow("GraphCoordination::simulateView::view", WINDOW_NORMAL);
	imshow("GraphCoordination::simulateView::view", fu);
	waitKey(0);
	*/
}

float Costmap::getPoseReward(Mat &mat){
float reward = 0;
	for(int i=0; i<exploreReward.cols; i++){
		for(int j=0; j<exploreReward.rows; j++){
			Point a(i,j);
			if(mat.at<uchar>(a) == 255 ){
				reward += exploreReward.at<float>(a);
			}
		}
	}
	return reward;
}

void Costmap::buildCellsPlot(){
	this->displayPlot= Mat::zeros(cells.size(),CV_8UC3);
	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point a(i,j);
			if(this->cells.at<short>(a) == this->obsFree){
				this->displayPlot.at<Vec3b>(a) = this->cObsFree;
			}
			else if(this->cells.at<short>(a) == this->frontier){
				this->displayPlot.at<Vec3b>(a) = this->cFrontier;
			}
			else if(this->cells.at<short>(a) == this->obsWall){
				this->displayPlot.at<Vec3b>(a) = this->cObsWall;
			}
			else{
				this->displayPlot.at<Vec3b>(a) = this->cUnknown;
			}
		}
	}
}

void Costmap::addAgentToPlot(Scalar color, vector<Point> myPath, Point cLoc){
	circle(displayPlot, cLoc, 2, color, -1);
	for(size_t i=1; i<myPath.size(); i++){
		Point a = myPath[i];
		Point b = myPath[i-1];
		line(displayPlot, a, b, color, 1);
	}
}

vector<Point> Costmap::aStarPath(Point sLoc, Point gLoc){

	if(sLoc == gLoc){
		vector<Point> totalPath;
		for(int i=0; i<4; i++){
			Point t = sLoc;
			totalPath.push_back(t);
		}
		return totalPath;
	}

	Mat cSet = Mat::zeros(cells.size(), CV_16S); // 1 means in closed set, 0 means not
	Mat oSet = Mat::zeros(cells.size(), CV_16S); // 1 means in open set, 0 means not
	Mat cameFromX = Mat::ones(cells.size(), CV_16S)*-1; // each square has a vector of the location it came from
	Mat cameFromY = Mat::ones(cells.size(), CV_16S)*-1; // each square has a vector of the location it came from

	Mat gScore = Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n
	Mat fScore = Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n

	vector<Point> oVec;
	oVec.push_back(sLoc);
	oSet.at<short>(sLoc) = 1; // starting node has score 0
	gScore.at<float>(sLoc)  = 0; // starting node in open set
	fScore.at<float>(sLoc) = sqrt(pow(sLoc.x-gLoc.x,2) + pow(sLoc.y-gLoc.y,2));
	fScore.at<float>(gLoc) = 1;

	bool foo = true;

	while(foo){
		/////////////////// this finds node with lowest fScore and makes current
		float min = INFINITY;
		int mindex = -1;

		for(size_t i=0; i<oVec.size(); i++){
			if(fScore.at<float>(oVec[i]) < min){
				min = fScore.at<float>(oVec[i]);
				mindex = i;
			}
		}

		Point cLoc = oVec[mindex];
		oVec.erase(oVec.begin() + mindex);
		oSet.at<short>(cLoc) = 0;
		cSet.at<short>(cLoc) = 1;

		//cout << "*** sLoc/cLoc/gLoc: " << sLoc << " / "<< cLoc << " / "<< gLoc << endl;

		/////////////////////// end finding current node
		if(pointCompare(cLoc, gLoc) ){ // if the current node equals goal, construct path
			vector<Point> totalPath;
			totalPath.push_back(gLoc);
			while( cLoc.x != sLoc.x || cLoc.y != sLoc.y ){ // work backwards to start
				Point tLoc(cameFromX.at<short>(cLoc), cameFromY.at<short>(cLoc));
				totalPath.push_back(tLoc); // append path
				cLoc.x = tLoc.x;
				cLoc.y = tLoc.y;
			}
			reverse(totalPath.begin(),totalPath.end());
			return totalPath;
		} ///////////////////////////////// end construct path

		// for nbrs
		int nx[8] = {-1,-1,-1,0,0,1,1,1};
		int ny[8] = {1,0,-1,1,-1,1,0,-1};

		for(int ni = 0; ni<8; ni++){
			Point nbr;
			nbr.x += cLoc.x + nx[ni];
			nbr.y += cLoc.y + ny[ni];
			if(pointOnMat(nbr, cells) ){
				//cout << "sLoc/cLoc/nbr/gLoc: " << sLoc << " / "<< cLoc << " / "<< nbr << " / "<< gLoc << endl;
				if(cSet.at<short>(nbr) == 1){ // has it already been eval? in cSet
					continue;
				}
				float ngScore = gScore.at<float>(cLoc) + sqrt(pow(cLoc.x-nbr.x,2) + pow(cLoc.y-nbr.y,2));//getEuclidianDistance(cLoc, nbr); // calc temporary gscore, estimate of total cost
				if(oSet.at<short>(nbr) == 0){
					oSet.at<short>(nbr) = 1;  // add nbr to open set
					oVec.push_back(nbr);
				}
				else if(ngScore >= gScore.at<float>(nbr) ){ // is temp gscore worse than stored g score of nbr
					continue;
				}
				cameFromX.at<short>(nbr) = cLoc.x;
				cameFromY.at<short>(nbr) = cLoc.y;

				gScore.at<float>(nbr) = ngScore;
				if(cells.at<short>(nbr) < 102){
					fScore.at<float>(nbr) = gScore.at<float>(nbr) + sqrt(pow(nbr.x-gLoc.x,2) + pow(nbr.y-gLoc.y,2));// +getEuclidianDistance(gLoc,nbr)
				}
				else{
					fScore.at<float>(nbr)= INFINITY;
				}
			}
		}
		/////////////// end condition for while loop, check if oSet is empty
		if(oVec.size() == 0){
			foo = false;
		}
	}
	vector<Point> totalPath;
	for(int i=0; i<4; i++){
		Point t = sLoc;
		totalPath.push_back(t);
	}
	return totalPath;
}

float Costmap::aStarDist(Point sLoc, Point gLoc){
	if(sLoc == gLoc){
		return 0;
	}

	Mat cSet = Mat::zeros(cells.size(), CV_16S); // 1 means in closed set, 0 means not
	Mat oSet = Mat::zeros(cells.size(), CV_16S); // 1 means in open set, 0 means not

	Mat gScore = Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n
	Mat fScore = Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n

	vector<Point> oVec;
	oVec.push_back(sLoc);
	oSet.at<short>(sLoc) = 1; // starting node has score 0
	gScore.at<float>(sLoc)  = 0; // starting node in open set
	fScore.at<float>(sLoc) = sqrt(pow(sLoc.x-gLoc.x,2) + pow(sLoc.y-gLoc.y,2));
	fScore.at<float>(gLoc) = 1;

	bool foo = true;

	while(foo){
		/////////////////// this finds node with lowest fScore and makes current
		float min = INFINITY;
		int mindex = -1;

		for(size_t i=0; i<oVec.size(); i++){
			if(fScore.at<float>(oVec[i]) < min){
				min = fScore.at<float>(oVec[i]);
				mindex = i;
			}
		}

		Point cLoc = oVec[mindex];
		oVec.erase(oVec.begin() + mindex);
		oSet.at<short>(cLoc) = 0;
		cSet.at<short>(cLoc) = 1;

		//cout << "*** sLoc/cLoc/gLoc: " << sLoc << " / "<< cLoc << " / "<< gLoc << endl;

		/////////////////////// end finding current node
		if(pointCompare(cLoc, gLoc) ){ // if the current node equals goal, construct path
			return gScore.at<float>(cLoc);
		} ///////////////////////////////// end construct path

		// for nbrs
		int nx[8] = {-1,-1,-1,0,0,1,1,1};
		int ny[8] = {1,0,-1,1,-1,1,0,-1};

		for(int ni = 0; ni<8; ni++){
			Point nbr;
			nbr.x += cLoc.x + nx[ni];
			nbr.y += cLoc.y + ny[ni];
			if(pointOnMat(nbr, cells) ){
				if(cSet.at<short>(nbr) == 1){ // has it already been eval? in cSet
					continue;
				}
				float ngScore = gScore.at<float>(cLoc) + sqrt(pow(cLoc.x-nbr.x,2) + pow(cLoc.y-nbr.y,2));//getEuclidianDistance(cLoc, nbr); // calc temporary gscore, estimate of total cost
				if(oSet.at<short>(nbr) == 0){
					oSet.at<short>(nbr) = 1;  // add nbr to open set
					oVec.push_back(nbr);
				}
				else if(ngScore >= gScore.at<float>(nbr) ){ // is temp gscore worse than stored g score of nbr
					continue;
				}

				gScore.at<float>(nbr) = ngScore;
				if(cells.at<short>(nbr) < 102){
					fScore.at<float>(nbr) = gScore.at<float>(nbr) + sqrt(pow(nbr.x-gLoc.x,2) + pow(nbr.y-gLoc.y,2));// +getEuclidianDistance(gLoc,nbr)
				}
				else{
					fScore.at<float>(nbr)= INFINITY;
				}
			}
		}
		/////////////// end condition for while loop, check if oSet is empty
		if(oVec.size() == 0){
			foo = false;
		}
	}

	return INFINITY;
}


