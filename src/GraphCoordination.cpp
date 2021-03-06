/*
 * GraphCoordination.cpp
 *
 *  Created on: Nov 7, 2016
 *      Author: andy
 */


#include "GraphCoordination.h"

using namespace cv;
using namespace std;

float getMatCount( Mat &m );
void prettyPrint2D(vector<vector<int> > &in);
void prettyPrint2D(vector<vector<float> > &in);

GraphCoordination::GraphCoordination(){}

void GraphCoordination::init(int obsRadius, int comRadius, vector<float> constants){
	this->constants.push_back(constants[2]);
	this->constants.push_back(constants[3]);
	this->constants.push_back(constants[4]);
	this->constants.push_back(constants[5]);


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

	temp = Mat::zeros(2*(comRadius + 1), 2*(comRadius + 1), CV_8UC1);
	cent.x = comRadius;
	cent.y = comRadius;
	circle(temp,cent,comRadius*0.9, Scalar(255));

	for(int i=0; i<temp.cols; i++){
		for(int j=0; j<temp.rows; j++){
			if(temp.at<uchar>(i,j,0) == 255){
				Point t(i-comRadius, j-comRadius);
				comPerim.push_back(t);
			}
		}
	}
	this->comRadius = comRadius;
}

Point GraphCoordination::relayPlanning( Costmap &costmap, Market &market, Point oLoc, Point &rLoc){

	/*
	cout << "market.roles: ";
	for(size_t i=0; i<market.roles.size(); i++){
		cout << market.roles[i] << ", ";
	}
	cout << endl;
	*/

	market.roles[market.myIndex] = 'e'; // make me an explorer, provides a null view
	Mat comMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
	int relayCount = getCoveredRelays(costmap, market, comMat, oLoc);
	int explorerCount = getCoveredExplorers( comMat, market );

	/*
	cout << "comArea / relays / explorers: " << getMatCount( comMat ) << " / " << relayCount << " / " << explorerCount << endl;
	namedWindow("main comMat", WINDOW_NORMAL);
	imshow("main comMat", comMat);
	waitKey(1);
	*/

	rReward = -INFINITY;
	rLoc = Point(-1,-1);

	// search for optimal pose
	market.roles[market.myIndex] = 'r'; // make me a relay to include my view!
	//over complete thinGraph poses
	for(size_t i=0;i < thinGraph.nodeLocations.size(); i++){
		if( comMat.at<uchar>(thinGraph.nodeLocations[i]) == 255){ // node is in comArea
			market.cLocs[market.myIndex] = thinGraph.nodeLocations[i]; // simulate my position as a relay
			Mat comMatTemp = comMat.clone();
			simulateCommunication(thinGraph.nodeLocations[i], comMatTemp, costmap);
			int relayCountTemp = getCoveredRelays( market, comMatTemp ); // get all relays covered by comGraph and generate comArea
			int explorerCountTemp = getCoveredExplorers( comMatTemp, market );

			float dComArea = getMatCount( comMatTemp ) - getMatCount( comMat );
			int dRelays = relayCountTemp - relayCount;
			int dExplorers = explorerCountTemp - explorerCount;

			float rI = constants[0]*sqrt(dComArea) + constants[1]*dRelays + constants[2]*dExplorers + constants[3]*costmap.aStarDist(market.cLocs[market.myIndex], thinGraph.nodeLocations[i]); // put in path length

			/*
			cout << "dComArea / dRelays / dExplorers: " << dComArea << " / " << dRelays << " / " << dExplorers << endl;
			cout << "rI: " << rI << endl;

			namedWindow("d comMat", WINDOW_NORMAL);
			imshow("d comMat", comMatTemp);
			waitKey(1);
			*/

			if(rI > rReward){
				rReward = rI;
				rLoc = thinGraph.nodeLocations[i];
			}
		}
	}
	return rLoc;
}

float getMatCount( Mat &m ){
	float c=0;
	for(int i=0;i<m.rows; i++){
		for(int j=0; j<m.cols; j++){
			Point t(i,j);

			if(m.at<uchar>(t) == 255){
				c++;
			}
		}
	}
	return c;
}


void GraphCoordination::simulateCommunication(Point pose, Mat &comMat, Costmap &costmap){

	for(size_t i=0; i<comPerim.size(); i++){
		Point v(comPerim[i].x + pose.x, comPerim[i].y + pose.y);
		LineIterator it(comMat, pose, v, 8, false);
		for(int i=0; i<it.count; i++, ++it){
			Point pp  = it.pos();
			if(costmap.cells.at<short>(pp) == costmap.obsWall){
				break;
			}
			else{
				comMat.at<uchar>(pp) = 255;
			}
		}
	}

	/*
	Mat fu = comMat.clone();
	circle(fu, pose, 2, Scalar(127), -1, 8);
	namedWindow("GraphCoordination::simulateCommunication::comMat", WINDOW_NORMAL);
	imshow("GraphCoordination::simulateCommunication::comMat", fu);
	waitKey(0);
	*/
}

int GraphCoordination::getCoveredExplorers( Mat &comMat, Market &market ){

	// find all agents in commo with the seed without using minus
	Mat tMat = comMat.clone();
	int cSet = 0;
	for(size_t i=0; i<market.cLocs.size(); i++){
		if(market.roles[i] == 'e' && comMat.at<uchar>(market.cLocs[i]) == 255 && i != market.myIndex){ // explorer and on the comMat
			cSet++;
			circle(tMat, market.cLocs[i], 2, Scalar(127), -1, 8);
		}
	}

	/*
	namedWindow("eMat", WINDOW_NORMAL);
	imshow("eMat", tMat);
	waitKey(1);
	*/

	return cSet;
}

int GraphCoordination::getCoveredRelays(Market &market, Mat &comMat ){

	// find all agents in commo with the seed without using minus
	Mat tMat = comMat.clone();

	int cSet = 0;
	for(size_t i=0; i<market.cLocs.size(); i++){
		if(market.roles[i] == 'r' && comMat.at<uchar>(market.cLocs[i]) == 255 && i != market.myIndex){ // relay and on the comMat
			cSet++;
			circle(tMat, market.cLocs[i], 2, Scalar(127), -1, 8);
		}
	}

	/*
	namedWindow("rMat", WINDOW_NORMAL);
	imshow("rMat", tMat);
	waitKey(1);
	*/

	return cSet;
}

int GraphCoordination::getCoveredRelays( Costmap &costmap, Market &market, Mat &comMat, Point oLoc ){

	// find all agents in commo with the seed without using minus
	vector<int> cSet;
	simulateCommunication( oLoc, comMat, costmap );

	bool changes = true;
	while( changes ){

		changes = false;

		for(size_t i=0; i<market.cLocs.size(); i++){
			if(market.roles[i] == 'r' && comMat.at<uchar>(market.cLocs[i]) == 255 && int(i) != market.myIndex){ // relay and on the comMat
				bool flag = true;
				for(size_t cs = 0; cs < cSet.size(); cs++){
					if( cSet[cs] == int(i) ){
						flag = false;
						break;
					}
				}
				if( flag ){ // not in cSet, add to cSet and comMat
					cSet.push_back( i );
					if( int(i) > market.myIndex ){
						simulateCommunication( market.gLocs[i], comMat, costmap );
					}
					simulateCommunication( market.cLocs[i], comMat, costmap );
					changes = true;
				}
			}
		}
	}

	/*
	namedWindow("GraphCoordination::getCoveredRelay::comMat", WINDOW_NORMAL);
	imshow("GraphCoordination::getCoveredRelay::comMat", comMat);
	waitKey(1);
	*/
	return int (cSet.size() );
}

bool GraphCoordination::commoCheck(Point aLoc, Point bLoc, Costmap &costmap){

	if( pow(aLoc.x-bLoc.x,2)+pow(aLoc.y-bLoc.y,2) > pow(comRadius,2)){
		return false;
	}

	Mat ta = Mat::zeros(costmap.cells.size(), CV_8UC1);
	LineIterator it(ta, aLoc, bLoc, 4, false);
	for(int i=0; i<it.count; i++, ++it){
		Point pp  = it.pos();
		//circle(ta, pp, 1, Scalar(255), -1, 8);
		if(pp.x >= costmap.cells.cols || pp.y >= costmap.cells.rows || costmap.cells.at<short>(pp) != costmap.obsFree){
			return false;
		}
	}
	return true;
}

void prettyPrint2D(vector<vector<int> > &in){
	for(size_t i=0; i<in.size(); i++){
		cout << "   ";
		for(size_t j=0; j<in[i].size(); j++){
			cout << in[i][j] << ", ";
		}
		cout << endl;
	}
}

void prettyPrint2D(vector<vector<float> > &in){
	for(size_t i=0; i<in.size(); i++){
		cout << "   ";
		for(size_t j=0; j<in[i].size(); j++){
			printf( "%.1f, ", in[i][j]);
		}
		cout << endl;
	}
}

GraphCoordination::~GraphCoordination() {

}

