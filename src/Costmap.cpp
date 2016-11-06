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
	this->cInfFree = {200,200,200};
	this->cObsWall = {0,0,0};
	this->cInfWall = {50,50,50};
	this->cUnknown = {127,127,127};
	this->cError = {255,0,0};

	this->obsFree = 1;
	this->infFree = 2;
	this->domFree = 3;

	this->unknown = 101;

	this->obsWall = 201;
	this->infWall = 202;
	this->inflatedWall = 203;

}

void Costmap::shareCostmap( Costmap &B ){
	for(int i=0; i<this->cells.cols; i++){
		for(int j=0; j<this->cells.rows; j++){
			Point a(i,j);
			// share cells
			if(this->cells.at<short>(a) != B.cells.at<short>(a) ){ // do we think the same thing?
				if(this->cells.at<short>(a) == this->unknown){
					this->cells.at<short>(a) = B.cells.at<short>(a); // if this->doesn't know, anything is better
				}
				else if(this->cells.at<short>(a) == this->infFree || this->cells.at<short>(a) == this->infWall){ // this->think its inferred
					if(B.cells.at<short>(a) == B.obsFree || B.cells.at<short>(a) == B.obsWall){ // B has observed
						this->cells.at<short>(a) = B.cells.at<short>(a);
					}
				}
			}
		}
	}
}

Costmap::~Costmap() {}

void Costmap::findFrontiers(){

	int dx[4] = {-1,1,0,0};
	int dy[4] = {0,0,-1,1};

	for(int i=1; i<cells.cols-1; i++){
		for(int j=1; j<cells.rows-1; j++){
			Point a(i,j);
			if(cells.at<short>(a) == obsFree){
				bool flag =false;
				for(int k=0; k<4; k++){
					Point b(a.x+dx[k], a.y+dy[k]);
					if(cells.at<short>(b) == unknown){
						flag = true;
					}
				}

				if(flag){
					cells.at<short>(a) = infFree;
				}
			}
		}
	}
}

vector<float> Costmap::nnGetObsFrntByQuadrant( Point cLoc ){

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
}

int getQuad( Point cLoc, Point t){
	if( t.x - cLoc.x >= 0 ){
		if( t.y  >= cLoc.y ){
			return 0;
		}
		else{
			return 3;
		}
	}
	else{
		if( t.y >= cLoc.y ){
			return 1;
		}
		else{
			return 2;
		}
	}
}

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

void Costmap::getRewardMat(float w[3], float e[2], float spread){
	// explore, search, map; dominated, breach; spread rate

	if(reward.empty()){
		reward = Mat::zeros(cells.size(), CV_32FC1);
	}

	if(w[1] > 0){ // spread if needed
		spreadSearchArea(spread);
		displaySearchReward();
	}

	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point a(i,j);
			float r[3] = {0,0,0};
			if(w[0] > 0){
				// explore reward
				if(cells.at<short>(a) == infFree){
					r[0] = 1;
				}
				else if(cells.at<short>(a) == domFree){
					r[0] = e[0];
				}
			}

			if(w[1] > 0){
				// search reward
				r[1] = searchReward.at<float>(a);
			}

			if(w[2] > 0){
				// explore reward
				if(1-occ.at<float>(a) > occ.at<float>(a) ){
					r[2] = occ.at<float>(a);
				}
				else{
					r[2] = 1-occ.at<float>(a);
				}
			}
			reward.at<float>(a) = w[0]*r[0] + w[1]*r[1] + w[2]*r[2];
		}
	}

	if(w[0] > 0){ // add hull breaches reward
		for(size_t k=0; k<hullBreaches.size(); k++){
			reward.at<float>(hullBreaches[k]) = e[1];
		}
	}
}

void Costmap::spreadSearchArea(float growthRate){

	if(searchReward.empty()){
		searchReward = Mat::zeros(cells.size(), CV_32FC1);
	}

	Mat ts = Mat::zeros(cells.size(), CV_32FC1);

	int sx[5] = {-1,0,0,0,1};
	int sy[5] = {0,1,0,-1,0};
	for(int i=1; i<cells.cols-1; i++){
		for(int j=1; j<cells.rows-1; j++){
			Point loc(i,j);
			if(cells.at<short>(loc) == obsFree){
				float sReward = 0;
				for(int a = 0; a<5; a++){
					Point nLoc(i+sx[a], j+sy[a]);
					sReward += searchReward.at<float>(nLoc);
				}
				if(sReward > (1-growthRate) * 5){
					ts.at<float>(loc) = 1;
				}
				else if(sReward < 0.025){
					ts.at<float>(loc) = 0;
				}
				else{
					ts.at<float>(loc) = sReward / ( (1-growthRate) * 5);
				}
			}
			else if(cells.at<short>(loc) == domFree || cells.at<short>(loc) == infFree){
				ts.at<float>(loc) = 1;
			}
		}
	}

	searchReward = ts.clone();

	for(size_t i=0; i<cellUpdates.size(); i++){
		if(cells.at<short>(cellUpdates[i]) != infFree){
			searchReward.at<float>(cellUpdates[i]) = 0;
		}
	}

}

void Costmap::displayThermalMat(Mat &mat){
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

			if(cells.at<short>(a) <= domFree){
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
	}

	namedWindow("Thermal Mat", WINDOW_NORMAL);
	imshow("Thermal Mat", tMat);
	waitKey(1);
}

void Costmap::displaySearchReward(){

	Mat msr = Mat::zeros( cells.size(), CV_8UC3);
	for(int i=0; i<searchReward.cols; i++){
		for(int j=0; j<searchReward.rows; j++){
			Point loc(i,j);
			if(cells.at<short>(loc) <= domFree){
				float sr = searchReward.at<float>(loc);
				if(sr > 1){
					sr = 1;
				}

				Vec3b color(0,0,0);
				if(sr < 0.125){
					color[1] = 255;
					color[2] = 0;
				}

				else if( sr > 0.875 ){
					color[1] = 0;
					color[2] = 255;
				}
				else{
					color[1] = round( 255 + (sr+0.125)*-255 );
					color[2] = round( (sr - 0.125)*340 );
				}

				msr.at<Vec3b>(loc) = color;
			}
		}
	}

	for(size_t i=0; i<cellUpdates.size(); i++){
		Vec3b white(255, 255, 255);
		msr.at<Vec3b>(cellUpdates[i]) = white;
	}

	namedWindow("Search Reward", WINDOW_NORMAL);
	imshow("Search Reward", msr);
	waitKey(1);
}

void Costmap::buildCellsPlot(){
	this->displayPlot= Mat::zeros(cells.size(),CV_8UC3);
	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point a(i,j);
			if(this->cells.at<short>(a) == this->obsFree){
				this->displayPlot.at<Vec3b>(a) = this->cObsFree;
			}
			else if(this->cells.at<short>(a) == this->infFree){
				this->displayPlot.at<Vec3b>(a) = this->cInfFree;
			}
			else if(this->cells.at<short>(a) == this->obsWall){
				this->displayPlot.at<Vec3b>(a) = this->cObsWall;
			}
			else if(this->cells.at<short>(a) == this->infWall){
				this->displayPlot.at<Vec3b>(a) = this->cInfWall;
			}
			else if(this->cells.at<short>(a) == this->inflatedWall){
				this->displayPlot.at<Vec3b>(a) = this->cInfWall;
			}
			else if(this->cells.at<short>(a) == this->unknown){
				this->displayPlot.at<Vec3b>(a) = this->cUnknown;
			}
			else{ // anything else, should never happen
				this->displayPlot.at<Vec3b>(a) = this->cError;
			}
		}
	}
}

void Costmap::buildOccPlot(){
	Mat temp= Mat::zeros(cells.size(),CV_8UC1);
	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point a(i,j);
			uchar shade;
			shade = round((1-occ.at<float>(a))*255);
			temp.at<uchar>(a) = shade;
		}
	}

	namedWindow("Occupancy Grid", WINDOW_NORMAL);
	imshow("Occupancy Grid", temp);
	waitKey(1);
}

void Costmap::addAgentToPlot(Scalar color, vector<Point> myPath, Point cLoc){
	circle(displayPlot, cLoc, 2, color, -1);
	for(size_t i=1; i<myPath.size(); i++){
		Point a = myPath[i];
		Point b = myPath[i-1];
		line(displayPlot, a, b, color, 1);
	}
}

vector<Point> Costmap::getImagePointsAt(Mat &image, int intensity){
	vector<Point> temp;
	for(int i=0; i<image.cols; i++){
		for(int j=0; j<image.rows; j++){
			if(image.at<uchar>(i,j,0) == intensity){
				Point t(i,j);
				temp.push_back(t);
			}
		}
	}
	return temp;
}

void Costmap::getDistGraph(){
	euclidDist = Mat::ones(cells.size(), CV_32FC1)*-1;
}

float Costmap::getEuclidianDistance(Point a, Point b){
	int dx = a.x - b.x;
	int dy = a.x - b.y;

	if(euclidDist.at<float>(dx,dy) == -1){
		euclidDist.at<float>(dx,dy) = sqrt(pow(dx,2) + pow(dy,2));
	}

	return(euclidDist.at<float>(dx,dy) );
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

	cerr << "into aStarDist" << endl;

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
			cerr << "out of aStarDist" << endl;
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


float Costmap::cumulativeAStarDist(Point sLoc, Point gLoc, Mat &cSet, vector<Point> &oSet, Mat &fScore, Mat &gScore){

	if(cSet.empty() == 1){
		cSet = Mat::zeros(cells.size(), CV_16SC1);

		fScore = Mat::zeros(cells.size(), CV_32F);
		gScore = Mat::zeros(cSet.size(), CV_32F);
		oSet.push_back(sLoc);
	}
	else if(cSet.at<short>(gLoc) == 255){
		return gScore.at<float>(gLoc);
	}

	// for nbrs
	int nx[8] = {-1,-1,-1,0,0,1,1,1};
	int ny[8] = {1,0,-1,1,-1,1,0,-1};

	Point nbrLoc(-1,-1);

	if(cells.at<short>(sLoc) > domFree){
		return INFINITY;
	}
	fScore.at<float>(sLoc) = getEuclidianDistance(gLoc,sLoc);

	while(oSet.size()){
		/////////////////// this finds node with lowest fScore and makes current
		float minF = INFINITY;
		int cIndex = -1;
		for(size_t i=0; i<oSet.size(); i++){
			if(fScore.at<float>(oSet[i]) < minF){
				minF = fScore.at<float>(oSet[i]);
				cIndex = i;
			}
		}

		Point cLoc = oSet[cIndex];
		float cGScore = gScore.at<float>(oSet[cIndex]);
		float cFScore = minF;

		cSet.at<short>(cLoc) = 255;
		oSet.erase(oSet.begin() + cIndex);

		/*
		cMat.at<short>(gLoc[0], gLoc[1]) = 127;
		namedWindow("A star", WINDOW_NORMAL);
		imshow("A star", cMat);
		waitKey(1);
		*/
		/////////////////////// end finding current node
		if(pointCompare(cLoc, gLoc) ){ // if the current node equals goal, construct path
			return cFScore;
		} ///////////////////////////////// end construct path

		for(int nbr = 0; nbr<8; nbr++){

			Point nbrLoc(cLoc.x + nx[nbr], cLoc.y + ny[nbr]);

			if(nbrLoc.x >= 0 && nbrLoc.x < cells.cols && nbrLoc.y >= 0 && nbrLoc.y < cells.rows){
				bool cFlag = true;

				if(cSet.at<short>(nbrLoc) == 255){ // has it already been eval? in cSet
					cFlag = false;
				}

				if(cFlag){
					if(this->cells.at<short>(nbrLoc) <= unknown){
						float tGScore = cGScore + getEuclidianDistance(nbrLoc, cLoc); // calc temporary gscore
						cFlag = true;
						for(size_t i=0; i<oSet.size(); i++){
							if(oSet[i] == nbrLoc){
								cFlag = false; // already in oSet, update score if found cheaper
								if(tGScore < gScore.at<float>(oSet[i]) ){
									gScore.at<float>(oSet[i]) = tGScore;
									fScore.at<float>(oSet[i]) = tGScore + getEuclidianDistance(gLoc,nbrLoc);
								}
								break;
							}
						}

						if(cFlag){ // not in oSet, so add
							oSet.push_back(nbrLoc);  // add nbr to open set
							gScore.at<float>(nbrLoc) = tGScore;
							fScore.at<float>(nbrLoc) = tGScore + getEuclidianDistance(gLoc,nbrLoc);
						}
					}
					else{
						cSet.at<short>(nbrLoc) = 255;
					}
				}
			}
		}
	}

	return INFINITY;
}


