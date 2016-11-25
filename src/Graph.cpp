/*
 * Graph.cpp
 *
 *  Created on: Nov 7, 2016
 *      Author: andy
 */


#include "Graph.h"

int getMinIndex(vector<float> value);
int getMaxIndex(vector<float> value);

Point findIntersection(Vec4i w1, Vec4i w2);
float distToLine(Vec4i w, Point a);
Point extendLine(Point a, Point m);
float distToLineSegment(Point p, Point v, Point w);
void clearArea(Mat &thinMat, Point loc, int grafSpacing);


using namespace cv;
using namespace std;

Graph::Graph(){
	nodeSpacing = 100;
	nbrSpacing = nodeSpacing;
}

void Graph::findStatesByGrid(Mat &inMat){
	for(int i=0; i<inMat.rows; i += nodeSpacing){
		for(int j=0; j<inMat.cols; j += nodeSpacing){
			if(inMat.at<uchar>(i,j) == 255){
				Point loc(i,j);
				nodeLocations.push_back(loc);
			}
		}
	}
}

void Graph::clearTransitions( int i ){
	for(size_t j = 0; j<nodeTransitions.size(); j++){
		nodeTransitions[j].erase( nodeTransitions[j].begin() + i);
	}
	nodeTransitions.erase( nodeTransitions.begin() + i);
}

void Graph::findStatesCityBlockDistance(Mat &thinMat){
	size_t i=0; // don't include cNode
	while(i<nodeLocations.size()){
		if(thinMat.at<uchar>(nodeLocations[i]) != 255){
			nodeLocations.erase(nodeLocations.begin() + i); // erase old nodes that shouldn't be nodes anymore
			clearTransitions(i);
		}
		else{
			clearArea(thinMat, nodeLocations[i], nodeSpacing); // clear area surrounding nodes that should be
		}
		i++;
	}

	/*
	namedWindow("Graph::thinGraph", WINDOW_NORMAL);
	imshow("Graph::thinGraph", thinMat);
	waitKey(0);
	*/

	for(int i=0; i<thinMat.cols; i++){ // find new nodes
		for(int j=0; j<thinMat.rows; j++){
			Point a(i,j);
			if(thinMat.at<uchar>(a) == 255){
				nodeLocations.push_back(a);
				clearArea(thinMat, a, nodeSpacing);
			}
		}
	}
}

void Graph::findStateTransitionsCityBlockDistance(Costmap &costmap){

	nodeTransitions.clear();
	for(size_t i=0; i<nodeLocations.size(); i++){
		vector<float> t;
		for(size_t j=0; j<nodeLocations.size(); j++){
			t.push_back(INFINITY);
		}
		nodeTransitions.push_back(t);
	}

	for(size_t i=0; i<nodeLocations.size(); i++){
		for(size_t j=i+1; j<nodeLocations.size(); j++){
			//cerr << i << "-i.loc: " << nodes[i].loc[0] << " , " << nodes[i].loc[1] << endl;
			//cerr << j << "-j.loc: " << nodes[j].loc[0] << " , " << nodes[j].loc[1] << endl;
			float d = costmap.getEuclidianDistance(nodeLocations[i],nodeLocations[j]);
			if(d < nbrSpacing && i != j){
				nodeTransitions[i][j] = d;
				nodeTransitions[j][i] = d;
			}
		}
	}
}

void Graph::findStateTransitionsByVisibility(Costmap &costmap){
	// make perimeter of viewing circle fit on image

	nodeTransitions.clear();
	for(size_t i=0; i<nodeLocations.size(); i++){
		vector<float> t;
		for(size_t j=0; j<nodeLocations.size(); j++){
			t.push_back(INFINITY);
		}
		nodeTransitions.push_back(t);
	}

	for(size_t i=0; i<nodeLocations.size(); i++){
		for(size_t j=i+1; j<nodeLocations.size(); j++){
			float d = costmap.getEuclidianDistance(nodeLocations[i],nodeLocations[j]);
			if(d < nbrSpacing && i != j){
				if(visibleLineCheck(costmap, nodeLocations[i],nodeLocations[j])){
					nodeTransitions[i][j] = d;
					nodeTransitions[j][i] = d;
				}
			}
		}
	}
}

bool Graph::visibleLineCheck(Costmap &costmap, Point pose, Point pt){

	Mat t = Mat::zeros(costmap.cells.size(), CV_8UC1);

	LineIterator it(t, pose, pt, 4, false);

	for(int i=0; i<it.count; i++, ++it){

		Point pp  = it.pos();

		if(costmap.cells.at<uchar>(pp) > costmap.obsFree){
			return false;
		}
	}

	return true;
}

void Graph::displayCoordMap(Costmap &costmap, bool displayNodes){
	Mat coordGraph = Mat::zeros(costmap.cells.size(), CV_8UC3);
	Scalar white;
	white[0] = 255; white[1] = 255; white[2] = 255;
	Scalar gray;
	gray[0] = 127; gray[1] = 127; gray[2] = 127;
	Scalar red;
	red[0] = 0; red[1] = 0; red[2] = 255;
	Scalar green;
	green[0] = 0; green[1] = 255; green[2] = 0;
	Scalar blue;
	blue[0] = 255; blue[1] = 0; blue[2] = 0;


	if(displayNodes){
		for(size_t i=0; i<nodeLocations.size(); i++){
			circle(coordGraph,nodeLocations[i],3,white,-1,8);
			//char str[50];
			//sprintf(str,"%d",i);
			//putText(coordGraph, str, temp, FONT_HERSHEY_PLAIN,0.5,green);
		}
	}


	if(nodeTransitions.size() == nodeLocations.size() && nodeTransitions[0].size() == nodeLocations.size()){
		for(size_t i=0; i<nodeLocations.size()-1; i++){
			for(size_t j=i+1; j<nodeLocations.size(); j++){
				cerr << "b" << endl;
				if(nodeTransitions[i][j] < INFINITY){
					cerr << "a" << endl;
					line(coordGraph, nodeLocations[i], nodeLocations[j], white, 1,8);
				}
			}
		}
	}

	/*
	if(cNode >= 0){
		Point cloc;
		cloc.x = nodeLocations[cNode][1];
		cloc.y = nodeLocations[cNode][0];
		circle(coordGraph,cloc,2,blue,-1,8);
	}
	*/

	namedWindow("Graph::coordGraph", WINDOW_NORMAL);
	imshow("Graph::coordGraph", coordGraph);
}

int Graph::findNearestNode(Point in, Costmap &costmap){
	float minDist = 5;
	int minIndex = -1;
	for(size_t i=0; i<nodeLocations.size(); i++){
		float a = costmap.getEuclidianDistance(in, nodeLocations[i]);
		if(a < minDist){
			 if( visibleLineCheck( costmap, in, nodeLocations[i]) ){
					minDist = a;
					minIndex = i;
			 }
		}
	}
	return minIndex;
}

float Graph::aStarDist(int sIndex, int gIndex, Costmap &costmap){
	vector<bool> cSet(nodeLocations.size(), false); // 1 means in closed set, 0 means not
	vector<bool> oSet(nodeLocations.size(), false); // 1 means in open set, 0 means not
	vector<float> gScore(nodeLocations.size(), INFINITY); // known cost from initial state to n
	vector<float> fScore(nodeLocations.size(), INFINITY); // gScore + heuristic score (dist to goal + imposed cost)

	oSet[sIndex] = true; // starting state in open set
	gScore[sIndex] = 0; // starting state has score 0
	fScore[sIndex] = gScore[sIndex] + costmap.getEuclidianDistance(nodeLocations[sIndex],nodeLocations[gIndex]); // calc score of open set

	//cerr << "Graph::aStarDist::sNode.loc: " << nodes[sIndex].loc[0] << ", " << nodes[sIndex].loc[1] << endl;
	//cerr << "Graph::aStarDist::gNode.loc: " << nodes[gIndex].loc[0] << ", " << nodes[gIndex].loc[1] << endl;

	int foo = 1;
	int finishFlag = 0;
	while(foo>0 && finishFlag == 0){
		/////////////////// this finds state with lowest fScore and makes current
		float minScore = INFINITY;
		int current = -1;

		for(size_t i=0;i<nodeLocations.size();i++){
			//cerr << "Graph::aStarDist::fScore: " << fScore[i] << endl;
			if(fScore[i] < minScore && oSet[i]){
				minScore = fScore[i];
				current = i;
			}
		}

		if(current == -1){
			cerr << "Graph::A* Dist::No solution found, empty oSet" << endl;
			return INFINITY;
		}
		else{
			float cfScore = fScore[current];
			float cgScore = gScore[current];

			//cerr << "Graph::aStarDist::current.loc: " << nodes[current].loc[0] << ", " << nodes[current].loc[1] << endl;
			oSet[current] = false;
			cSet[current] = true;
			/////////////////////// end finding current state
			if(current == gIndex){ // if the current state equals goal, then return the distance to the goal
				return cfScore;
			} ///////////////////////////////// end construct path
			for(size_t nbr=0; nbr<nodeLocations.size(); nbr++){

				if(nodeTransitions[current][nbr] != INFINITY && cSet[nbr] == false){

					float tgScore = cgScore + costmap.getEuclidianDistance(nodeLocations[current], nodeLocations[nbr]); // calc temporary gscore
					if(oSet[nbr]){
						if(cgScore < tgScore){
							gScore[nbr] = tgScore;
							fScore[nbr] = gScore[nbr] + costmap.getEuclidianDistance(nodeLocations[nbr], nodeLocations[gIndex]);
						}
					}
					else{
						oSet[nbr] = true;
						gScore[nbr] = tgScore;
						fScore[nbr] = cgScore + costmap.getEuclidianDistance(nodeLocations[nbr], nodeLocations[gIndex]);
					}
				}
			}
			/////////////// end condition for while loop, check if oSet is empty
			foo = oSet.size();
		}
	}
	cerr << "Graph::A* Dist::No solution found, reached end of function" << endl;
	return INFINITY;
}

void Graph::createThinGraph(Costmap &costmap, int nodeSpacing, int nbrSpacing){

	this->nodeSpacing = nodeSpacing;
	this->nbrSpacing = nbrSpacing;
	// 1 = observed free
	// 2 = inferred free
	// 3 = dominated free
	// 101 = observed obstacle
	// 102 = inferred obstacle
	// 201 unknown


	// get Mat of all observed or inferred free cells to make travel graph
	Mat freeMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
	for(int i = 0; i<costmap.cells.cols; i++){
		for(int j =0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if(costmap.cells.at<short>(a) == costmap.obsFree || costmap.cells.at<short>(a) == costmap.frontier){// ||costmap.cells.at<short>(a) == costmap.unknown ){ // observed free space or inferred free space - to make travel graph
				freeMat.at<uchar>(a) = 255;
			}
		}
	}

	threshold(freeMat, freeMat,10,255,CV_THRESH_BINARY);
	thinning(freeMat, thinMat);

	Mat tMat = thinMat.clone();

	// add nodes
	nodeLocations.clear();
	findStatesCityBlockDistance(tMat); // add them to graf;
	//findStateTransitionsCityBlockDistance(costmap); // find connections in graf; // TODO add line checking to this for intersections and increase distance
	//findStatesByGrid(freeMat);
	//findStateTransitionsByVisibility(costmap);
	//findCNodeTransitions(costmap);
	//mergeStatesBySharedNbr(); // TODO make this work, currently runs forever

}

void Graph::downSample( Mat &oMat, Mat &nMat, Costmap &costmap){

	int dx[4] = {0,1,0,1};
	int dy[4] = {0,0,1,1};

	for(int i=0; i<costmap.cells.cols/2-2; i++){
		for(int j=0; j<costmap.cells.rows/2-2; j++){
			Point n(i,j);
			bool flag = true;
			for(int k=0; k<4; k++){
				Point p(2*i+dx[k], 2*j+dy[k]);

				if(costmap.cells.at<short>(p) >= costmap.unknown){
					flag = false;
				}
			}
			if(flag){
				nMat.at<uchar>(n) = 255;
			}
			else{
				nMat.at<uchar>(n) = 0;
			}
		}
	}
}

void Graph::pruneThinMat( Costmap &costmap ){

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point p(i,j);
			if(thinMat.at<uchar>(p) == 255 && costmap.cells.at<short>(p) >= costmap.unknown){
				thinMat.at<uchar>(p) = 0;
			}
		}
	}
}

void Graph::thinning(const Mat& src, Mat& dst){
 	//https://github.com/bsdnoobz/zhang-suen-thinning/blob/master/thinning.cpp
     dst = src.clone();
     dst /= 255;         // convert to binary image

     cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
     cv::Mat diff;

     do {
         thinningIteration(dst, 0);
         thinningIteration(dst, 1);
         cv::absdiff(dst, prev, diff);
         dst.copyTo(prev);
     }
     while (cv::countNonZero(diff) > 0);

     dst *= 255;
}

void Graph::thinningIteration(Mat& img, int iter){
 	//https://github.com/bsdnoobz/zhang-suen-thinning/blob/master/thinning.cpp
     CV_Assert(img.channels() == 1);
     CV_Assert(img.depth() != sizeof(uchar));
     CV_Assert(img.rows > 3 && img.cols > 3);

     cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

     int x, y;
     uchar *pAbove;
     uchar *pCurr;
     uchar *pBelow;
     uchar *nw, *no, *ne;    // north (pAbove)
     uchar *we, *me, *ea;
     uchar *sw, *so, *se;    // south (pBelow)

     uchar *pDst;

     // initialize row pointers
     pAbove = NULL;
     pCurr  = img.ptr<uchar>(0);
     pBelow = img.ptr<uchar>(1);

     for (y = 1; y < img.rows-1; ++y) {
         // shift the rows up by one
         pAbove = pCurr;
         pCurr  = pBelow;
         pBelow = img.ptr<uchar>(y+1);

         pDst = marker.ptr<uchar>(y);

         // initialize col pointers
         no = &(pAbove[0]);
         ne = &(pAbove[1]);
         me = &(pCurr[0]);
         ea = &(pCurr[1]);
         so = &(pBelow[0]);
         se = &(pBelow[1]);

         for (x = 1; x < img.cols-1; ++x) {
             // shift col pointers left by one (scan left to right)
             nw = no;
             no = ne;
             ne = &(pAbove[x+1]);
             we = me;
             me = ea;
             ea = &(pCurr[x+1]);
             sw = so;
             so = se;
             se = &(pBelow[x+1]);

             int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) +
                      (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) +
                      (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                      (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
             int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
             int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
             int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

             if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0){
                 pDst[x] = 1;
             }
         }
     }

     img &= ~marker;
}

void clearArea(cv::Mat &thinMat, Point loc, int grafSpacing){
	circle(thinMat, loc, grafSpacing, Scalar(0), -1, 8);
}

float cityBlockDist(vector<int> a, vector<int> b){
	float d = abs(a[0] - b[0]) + abs(a[1]+b[1]);
	return d;
}

float euclidianDist(vector<int> a, vector<int> b){
	float d = sqrt(pow(a[0]-b[0],2) + pow(a[1] - b[1],2));
	return d;
}

int getMaxIndex(vector<float> value){
	int maxdex;
	float maxValue = -INFINITY;
	for(int i=0; i<(int)value.size(); i++){
		if(value[i] > maxValue){
			maxdex = i;
			maxValue  = value[i];
		}
	}
	return maxdex;
}

int getMinIndex(vector<float> value){
	int mindex;
	float minValue = INFINITY;
	for(int i=0; i<(int)value.size(); i++){
		if(value[i] < minValue){
			mindex = i;
			minValue  = value[i];
		}
	}
	return mindex;
}

float distToLineSegment(Point p, Point v, Point w){
	float l = pow(v.x - w.x,2) + pow(v.y-w.y,2);
	if(l==0){ return sqrt(pow(v.x - p.x,2) + pow(v.y-p.y,2) ); } // v==w
	float t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l;
	if(t > 1){
		t = 1;
	}
	else if(t < 0){
		t = 0;
	}
	int xl = v.x + t * (w.x - v.x);
	int yl = v.y + t * (w.y - v.y);
	return sqrt( pow(p.x - xl,2) + pow(p.y-yl,2) );
}

Point extendLine(Point a, Point m){
	float dx = m.x - a.x;
	float dy = m.y - a.y;
	Point p;
	p.x = m.x + dx;
	p.y = m.y + dy;
	return(p);
}

float distToLine(Vec4i w, Point a){
	float x1 = w[0];
	float y1 = w[1];
	float x2 = w[2];
	float y2 = w[3];

	float x0 = a.x;
	float y0 = a.y;

	float denom = sqrt(pow(x2-x1,2) + pow(y2-y1,2));
	if(denom != 0){
		float dist = abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))/denom;
		return dist;
	}
	else{
		return(-1);
	}
}

Point findIntersection(Vec4i w1, Vec4i w2){
	float x1 = w1[0];
	float y1 = w1[1];
	float x2 = w1[2];
	float y2 = w1[3];

	float x3 = w2[0];
	float y3 = w2[1];
	float x4 = w2[2];
	float y4 = w2[3];

	float denom = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);

	if(denom != 0){
		Point p;
	    p.x = ((x1*y2-y1*x2)*(x3-x4) - (x3*y4-y3*x4)*(x1-x2))/denom;
	    p.y = ((x1*y2-y1*x2)*(y3-y4) - (x3*y4-y3*x4)*(y1-y2))/denom;
	    return(p);
	}
	else{
		Point p;
		p.x = -1;
		p.y = -1;
		return(p);
	}
}

Graph::~Graph(){}

void Graph::updateThinGraph(Costmap &costmap, int nodeSpacing, int nbrSpacing){
	this->nodeSpacing = nodeSpacing;
	this->nbrSpacing = nbrSpacing;
	// 1 = observed free
	// 2 = inferred free
	// 3 = dominated free
	// 101 = observed obstacle
	// 102 = inferred obstacle
	// 201 unknown

	// get Mat of all observed or inferred free cells to make travel graph
	Mat freeMat = Mat::zeros(costmap.cells.size(), CV_8UC1);

	for(size_t i=0; i<costmap.cells.cols; i++){
		for(size_t j=0; j<costmap.cells.rows; j++){
			if(costmap.cells.at<uchar>(i,j) == costmap.obsFree || costmap.cells.at<uchar>(i,j) == costmap.unknown ||costmap.cells.at<uchar>(i,j) == costmap.frontier ){
				freeMat.at<uchar>(i,j) = 255;
			}
		}
	}
	int limits2[4] = {2, costmap.cells.cols-3, 2, costmap.cells.rows-3};


	/*
	int limits[4] = {costmap.cells.size(), 0, costmap.cells[0].size(), 0};
	for(size_t i = 0; i<costmap.cellUpdates.size(); i++){
		int x = costmap.cellUpdates[i][0];
		int y = costmap.cellUpdates[i][1];
		freeMat.at<uchar>(x,y) = 255;

		if(x < limits[0]){
			limits[0] = x;
		}
		else if(x > limits[1]){
			limits[1] = x;
		}
		if(y < limits[2]){
			limits[2] = y;
		}
		else if(y > limits[3]){
			limits[3] = y;
		}
	}
	*/


	namedWindow("Graph::createThinGraph::freeMat", WINDOW_NORMAL);
	imshow("Graph::createThinGraph::freeMat", freeMat);
	waitKey(1);

	if(thinMat.empty()){
		thinMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
	}

	bitwise_or(freeMat, thinMat, freeMat);

	namedWindow("Graph::createThinGraph::freeMat", WINDOW_NORMAL);
	imshow("Graph::createThinGraph::freeMat", freeMat);
	waitKey(1);

	// get the thin free mat
	thinning(freeMat, thinMat);

	/*
	namedWindow("Graph::createThinGraph::thinMat", WINDOW_NORMAL);
	imshow("Graph::createThinGraph::thinMat", thinMat);
	waitKey(1);
	*/

	Mat tThinMat = thinMat.clone();

	// add nodes
	nodeLocations.clear();
	findStatesCityBlockDistance(tThinMat); // add them to graf;
	//findStateTransitionsCityBlockDistance(costmap); // find connections in graf; // TODO add line checking to this for intersections and increase distance
	//findStatesByGrid(freeMat);
	//findStateTransitionsByVisibility(costmap);
	//findCNodeTransitions(costmap);
	//mergeStatesBySharedNbr(); // TODO make this work, currently runs forever

}


