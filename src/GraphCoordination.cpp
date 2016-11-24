/*
 * GraphCoordination.cpp
 *
 *  Created on: Nov 7, 2016
 *      Author: andy
 */


#include "GraphCoordination.h"

using namespace cv;
using namespace std;

void buildGreedyTours(vector<vector<int> > &tours, vector<int> &oSet, vector<vector<float> > &distGraph);
void marketBids(vector<vector<int> > &tours, vector<int> &oSet, vector<int> &goalPoses, vector<float> &goalBids);
void getBids(vector< vector<int> > &tours, vector<int> &oSet, vector<vector<float> > distGraph, vector<int> &goalPoses, vector<float> &goalBids);
float getTourLength(vector<int> &tour, vector<vector<float> > &distGraph);
void swapPosesBetweenAgents(vector<vector<int> > &tours);
void transferPoses(vector<vector<int> > &tours);
void swapEntireTours(vector<vector<int> > &tours);
void evolveIndividualTours(vector<vector<int> > &tours, vector<vector<float> > &distGraph);
void evoluAlgTSP(vector<int> &tour, vector<vector<float> > &distGraph);
vector<vector<int> > tspEvolveTours(vector<vector<int> > &tours, vector<vector<float> > &distGraph);
void tspRecursiveSolver( vector<int> tour, float tourLength, vector<vector<float> > &distGraph, vector<int> oSet, vector<int> &minTour, float &minLength);
vector<int> getOpenPoses(vector<vector<int> > &tours, vector<vector<float> > &distGraph);

void prettyPrint2D(vector<vector<int> > &in);
void prettyPrint2D(vector<vector<float> > &in);

/*
void bruteForceTSP(vector<int> &tour, vector<vector<float> > &distGraph);
void bruteForce3(vector<int> tour, vector<vector<float> > distGraph);
void bruteForce4(vector<int> tour, vector<vector<float> > distGraph);
void bruteForce5(vector<int> tour, vector<vector<float> > distGraph);
void bruteForce6(vector<int> tour, vector<vector<float> > distGraph);
void bruteForce7(vector<int> tour, vector<vector<float> > distGraph);
*/

GraphCoordination::GraphCoordination(){
	nPullsEvolvePoseGraph = 1000;

	minObsForPose = 5;

	int obsRadius = 40;
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

void GraphCoordination::marketPoses( Costmap &costmap, Market &market ){

	cerr << "GraphCoordination::marketPoses::A" << endl;

	// am I closer to any poses in market than the current purchaser?
	for(size_t i=0; i<market.gLocs.size(); i++){
		if( i != market.myIndex ){
			bool flag = false;
			if( market.gLocs[i].x > 0 && market.gLocs[i].y > 0){
				if(sqrt(pow(market.cLocs[market.myIndex].x - market.gLocs[i].x,2) + pow(market.cLocs[market.myIndex].y - market.gLocs[i].y,2)) <  market.costs[i]){
					if(costmap.aStarDist(market.gLocs[i], market.cLocs[market.myIndex]) > market.costs[i]){
						cerr << "out of aStarDist: true" << endl;
						flag = true; // I am not a* closer
					}
					else{
						cerr << "out of aStarDist: false********" << endl;
					}
				}
				else{ // I am not euclidian closer
					flag = true;
				}
			}
			if(flag){ // they are closer, remove all reward from their goals
				Mat nView = Mat::zeros(costmap.cells.size(), CV_8UC1);
				simulateObservation( market.gLocs[i], nView, costmap);

				for(int j=0; j<costmap.exploreReward.cols; j++){
					for(int k=0; k<costmap.exploreReward.rows; k++){
						Point a(j,k);
						if( nView.at<uchar>(a) == 255){
							costmap.exploreReward.at<float>(a) = 0;
						}
					}
				}
			}
		}
	}
	//

	//cerr << "GraphCoordination::marketPoses::B" << endl;


	vector<float> poseRewards;
	vector<float> poseDistances;
	vector<bool> trueDist;
	vector<float> poseValue;

	float maxVal = -INFINITY;
	int maxPose = -1;

	//cerr << "GraphCoordination::marketPoses::C" << endl;

	Mat gView = Mat::zeros(costmap.cells.size(), CV_8UC1);
	//cerr << "PoseGraph.nodeLocations: ";
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		//cerr << poseGraph.nodeLocations[i] << ", ";
		Mat lView = Mat::zeros( costmap.cells.size(), CV_8UC1);
		simulateObservation( poseGraph.nodeLocations[i], lView, costmap );
		bitwise_or(gView, lView, gView);
	}
	//cerr << endl;

	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		//poseRewards.push_back( getDiscountedRewards( costmap, poseGraph.nodeLocations, i, globalReward ) );
		poseRewards.push_back( observedReward(poseGraph.nodeObservations[i], costmap.exploreReward) );
		if(poseRewards.back() < 1){
			poseRewards.back() = -INFINITY;
		}

		poseDistances.push_back( sqrt(pow(market.cLocs[market.myIndex].x-poseGraph.nodeLocations[i].x,2) + pow(market.cLocs[market.myIndex].y-poseGraph.nodeLocations[i].y,2) ));
		trueDist.push_back( false );

		poseValue.push_back( poseRewards.back() - 0.1*pow(poseDistances.back(),2) );

		//cout << "index, location, value, rewards, distance: " << i << ", " << poseGraph.nodeLocations[i] << ", " << poseValue.back() << ", " << poseRewards.back() << ", " << poseDistances.back() << endl;

		if(poseValue.back() > maxVal){
			maxVal = poseValue.back();
			maxPose = i;
		}
	}

	//cerr << "GraphCoordination::marketPoses::D" << endl;

	while( true ){ // find best pose

		if( trueDist[maxPose]){
			break;
		}

		poseDistances[maxPose] = costmap.aStarDist(market.cLocs[market.myIndex], poseGraph.nodeLocations[maxPose]);
		poseValue[maxPose] = poseRewards[maxPose] - 0.1*pow(poseDistances[maxPose],2);
		//cerr << "GraphCoordination::marketPoses::poseValue[maxPose]: " << poseValue[maxPose] << endl;
		trueDist[maxPose] = true;

		maxVal = -INFINITY;
		maxPose = -1;
		for(size_t i=0; i<poseValue.size(); i++){
			if(poseValue[i] > maxVal){
				maxPose = i;
				maxVal = poseValue[i];
			}
		}
		//cerr << "GraphCoordination::marketPoses::D5" << endl;


		cout << "maxPose: index, value, rewards, distance: " << maxPose << ", " << poseValue[maxPose] << ", " << poseRewards[maxPose] << ", " << poseDistances[maxPose] << endl;
	}

	//cerr << "GraphCoordination::marketPoses::gValue: " << poseValue[maxPose] << endl;
	//cerr << "GraphCoordination::marketPoses::goalPose: " << poseGraph.nodeLocations[maxPose] << endl;

	market.gLocs[market.myIndex] = poseGraph.nodeLocations[maxPose];
	market.costs[market.myIndex] = poseDistances[maxPose];

	//cerr << "GraphCoordination::marketPoses::Z" << endl;
}

void GraphCoordination::simulateNULLObservation(Point pose, Mat &resultingView, Costmap &costmap){
	// make perimeter of viewing circle fit on image

	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + pose.x, viewPerim[i].y + pose.y);
		LineIterator it(resultingView, pose, v, 8, false);
		for(int i=0; i<it.count; i++, ++it){

			Point pp  = it.pos();

			if(costmap.cells.at<short>(pp) == costmap.obsWall){
				break;
			}
			else{
				resultingView.at<uchar>(pp) = 0;
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

void GraphCoordination::getViews(Costmap &costmap, vector<int> cPoses, Mat &cView, float &cReward, vector<Point> &poses, vector<Mat> &views, vector<float> poseRewards, float &gReward){

	Mat gView = Mat::zeros(costmap.cells.size(), CV_8UC1);

	cerr << "cPoses: ";
	for(size_t i=0; i<cPoses.size(); i++){
		Mat t = Mat::zeros(costmap.cells.size(), CV_8UC1);
		simulateObservation(poses[ cPoses[i] ], t, costmap);
		cerr << poses[ cPoses[i] ] << ", ";
		bitwise_or(cView, t, cView);
	}
	cerr << endl;
	cReward = observedReward(cView, costmap.exploreReward);

	for(size_t i=0; i<poses.size(); i++){
		Mat t = Mat::zeros(costmap.cells.size(), CV_8UC1);
		simulateObservation(poses[i], t, costmap);
		views.push_back(t);
		poseRewards.push_back(observedReward( t, costmap.exploreReward ) );
		bitwise_or(gView, t, gView);
	}
	gReward = observedReward(gView, costmap.exploreReward);

	/*
	cerr << "gReward: " << gReward << endl;
	namedWindow("gView", WINDOW_NORMAL);
	imshow("gView", gView);
	waitKey(1);
	*/
}

void GraphCoordination::plotPoses( Costmap &costmap, vector<int> &cPoses, vector<int> &oPoses, vector<Mat> &views, vector<Point> &poses){

	Mat gView = Mat::zeros(views[0].size(), CV_8UC1);

	for(size_t i=0; i<cPoses.size(); i++){
		bitwise_or(gView, views[cPoses[i] ], gView);
	}

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if( gView.at<uchar>(a) > 127 && costmap.exploreReward.at<float>(a) == 0){
				gView.at<uchar>(a) = 0;
			}
		}
	}


	for(size_t i=0; i<oPoses.size(); i++){
		circle(gView, poses[ oPoses[i] ], 2, Scalar(100), -1, 8);
	}

	for(size_t i=0; i<cPoses.size(); i++){
		circle(gView, poses[ cPoses[i] ], 2, Scalar(200), -1, 8);
	}


	namedWindow("Pose Set", WINDOW_NORMAL);
	imshow("Pose Set", gView);
	waitKey(1);
}

void GraphCoordination::findPosesEvolution(Costmap &costmap){

	cout << "GraphCoordination::findPosesEvolution::A" << endl;

	// get graph observations and global reward
	vector<int> oPoses, cPoses, bPoses;
	vector<Point> poseLocations;
	vector<Mat> poseViews;
	vector<float> poseRewards;

	// add old nodes and get view
	Mat cView = Mat::zeros(costmap.cells.size(), CV_8UC1);
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		poseLocations.push_back( poseGraph.nodeLocations[i] );
		Mat tView = Mat::zeros(costmap.cells.size(), CV_8UC1);
		simulateObservation(poseGraph.nodeLocations[i], tView, costmap);
		poseViews.push_back(tView);
		float tReward = observedReward( tView, costmap.exploreReward );
		poseRewards.push_back( tReward );
		if( tReward > 0 ){
			cPoses.push_back(i);
			bitwise_or(cView, tView, cView);
		}
	}

	namedWindow("cView", WINDOW_NORMAL);
	imshow("cView", cView);
	waitKey(0);
	float cReward = observedReward( cView, costmap.exploreReward );
	Mat gView = Mat::ones(costmap.cells.size(), CV_8UC1)*255;
	float gReward = observedReward( gView, costmap.exploreReward );

	// add potential new nodes
	for(size_t i=0; i<thinGraph.nodeLocations.size(); i++){
		oPoses.push_back( cPoses.size() + i );
		poseLocations.push_back( thinGraph.nodeLocations[i] );
		poseRewards.push_back(-1);
		Mat tView = Mat::zeros(costmap.cells.size(), CV_8UC1);
		poseViews.push_back(tView);
	}

	// any poses with 0 reward removed from cPoses
	for(size_t i=0; i<cPoses.size(); i++){
		int t = cPoses[i];
		if( poseRewards[ t ] <= 0){
			cPoses.erase(cPoses.begin() + i);
			oPoses.push_back( t );
		}
	}

	cout << "GraphCoordination::findPosesEvolution::C::cReward / gReward: " << cReward << " / " << gReward << endl;


	// add more nodes to pose graph if not fully observing world

	clock_t tStart1 = clock();
	int iter = 0;
	while(cReward < gReward && iter < 500){ // add states that increase the observed are until % is observed
		iter++;
		int c = rand() % oPoses.size();
		int index = oPoses[c];
		if(poseRewards[index] == -1){
			simulateObservation(poseLocations[index], poseViews[index], costmap);
			poseRewards[index] = observedReward( poseViews[index], costmap.exploreReward );
		}

		Mat tView;
		bitwise_or(cView, poseViews[ index ], tView);
		float tReward = observedReward(tView, costmap.exploreReward);

		if(tReward > cReward){
			cPoses.push_back( index );

			cView = tView;
			cReward = tReward;

			oPoses.erase(oPoses.begin() + c);
		}
	}
	printf("Time taken to create get poses: %.2fs\n", (double)(clock() - tStart1)/CLOCKS_PER_SEC);

	//cerr << "GraphCoordination::findPosesEvolution::D::cReward / gReward: " << cReward << " / " << gReward << endl;
	int cPost = cPoses.size();
	iter = 0;
	while( iter < cPost ){
		int dP = cPoses[iter];

		Mat dMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
		for(size_t i=0; i<cPoses.size(); i++){
			if( int(i) != iter ){
				bitwise_or(dMat, poseViews[i], dMat);
			}
		}

		float dReward = observedReward( dMat, costmap.exploreReward );

		if( dReward == gReward ){
			cPoses.erase(cPoses.begin() + iter);
			oPoses.push_back( dP );
			cPost--;
			cerr << "erased dP" << endl;
		}
		iter++;
	}

	plotPoses( costmap, cPoses, oPoses, poseViews , poseLocations);
	waitKey(1);

	poseGraph.nodeLocations.clear();
	poseGraph.nodeObservations.clear();
	for(size_t i=0; i<cPoses.size(); i++){
		poseGraph.nodeLocations.push_back( poseLocations[ cPoses[i] ] );
		poseGraph.nodeObservations.push_back( poseViews[ cPoses[i] ] );
	}

	cerr << "GraphCoordination::findPosesEvolution::E: " << poseGraph.nodeLocations.size() << endl;
}

void GraphCoordination::getOset(vector<int> &oStates, vector<int> &cStates, vector<int> &workingSet){
	for(size_t i = 0; i<workingSet.size(); i++){
		bool flag = true;
		for(size_t j=0; j<cStates.size(); j++){
			if(workingSet[i] == cStates[j]){
				flag = false;
				break;
			}
		}
		if(flag){
			oStates.push_back(workingSet[i]);
		}
	}
}

float GraphCoordination::observedReward(Mat &observed, Mat &reward){
	// get Mat entropy
	float r = 0;
	for(int i=0; i<observed.cols; i++){
		for(int j=0; j<observed.rows; j++){
			Point a(i,j);
			if(observed.at<uchar>(a) > 0){
				r += reward.at<float>(a);
			}
		}
	}
	return r;
}

void GraphCoordination::simulateObservation(Point pose, Mat &resultingView, Costmap &costmap){
	// make perimeter of viewing circle fit on image

	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + pose.x, viewPerim[i].y + pose.y);
		LineIterator it(resultingView, pose, v, 8, false);
		for(int i=0; i<it.count; i++, ++it){

			Point pp  = it.pos();

			if(costmap.cells.at<short>(pp) == costmap.obsWall){
				break;
			}
			else{// if(costmap.cells.at<short>(x0, y0) == costmap.infFree){
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

Point GraphCoordination::posePathPlanningTSP(Graph &graph, Costmap &costmap, vector<Point> &agentLocs, int &myIndex){
	Point gLoc = agentLocs[myIndex];

	findPoseGraphTransitions(graph, costmap); // get A* dist
	cout << "GraphCoordination::found poseGraphTransitions" << endl;

	displayPoseGraph(costmap);
	waitKey(10);

	tspPoseFullTourPlanner(agentLocs, myIndex);
	cerr << "GraphCoordination::out of tspPoseToursPlanner" << endl;

	displayPoseTours(costmap);
	waitKey(1);

	cout << "TSPPosePathPlanning::returning tspPoseTours[1]: " << poseGraph.nodeLocations[tspPoseTours[myIndex][1]].x << ", " << poseGraph.nodeLocations[tspPoseTours[myIndex][1]].y << endl;
	return poseGraph.nodeLocations[ tspPoseTours[myIndex][1] ];
}


void GraphCoordination::tspPoseFullTourPlanner(vector<Point> &agentLocs, int &myIndex){

	cout << "GraphCoordination::tspPoseFullTourPlanner::seeding tours" << endl;
	// initialize + seed paths
	if( tspPoseTours.size() > 1){
		for(size_t i=0; i<tspPoseToursLocations.size(); i++){
			if( tspPoseTours[i].size() > 1){
				vector<int> tempPath;
				tempPath.push_back(i); // reseed cNode
				cerr << "a" << endl;
				for(size_t k=1; k<tspPoseToursLocations[i].size(); k++){
					cerr << "b" << endl;
					for(size_t j=0; j<poseGraph.nodeLocations.size(); j++){
						cerr << "tspPoseToursLocations[i].size(): " << tspPoseToursLocations[i].size() << endl;
						cerr << "k: " << k << endl;
						cerr << "tspPoseToursLocations[i][k].x: " << tspPoseToursLocations[i][k].x << endl;
						cerr << "poseGraph.nodeLocations[j].x: " << poseGraph.nodeLocations[j].x << endl;
						if(tspPoseToursLocations[i][k] == poseGraph.nodeLocations[j]){
							cerr << "d" << endl;
							tempPath.push_back(j);
							break;
						}
					}
				}
				cerr << "e" << endl;
				tspPoseTours[i] = tempPath;
			}
		}
		cerr << "f" << endl;
	}
	else{ // tspPoseTours.size() == 0, so initialize
		for(size_t i=0; i<agentLocs.size(); i++){ // first time, initialize tspPoseTours
			vector<int> t(1,i); // seed in cNodes
			tspPoseTours.push_back(t);
		}
	}

	cout << "GraphCoordination::tspPoseFullTourPlanner::seeded tours" << endl;;
	prettyPrint2D(tspPoseTours);
	waitKey(0);

	// get openSet
	vector<int> openPoses = getOpenPoses(tspPoseTours, poseGraph.nodeTransitions);

	cout << "GraphCoordination::tspPoseFullTourPlanner::got open poses" << endl;
	cout << "   ";
	for(size_t i=0; i<openPoses.size(); i++){
		cout << openPoses[i] << ", ";
	}
	cout << endl;

	// greedily add openPoses to tours
	buildGreedyTours(tspPoseTours, openPoses, poseGraph.nodeTransitions);

	cout << "GraphCoordination::tspPoseFullTourPlanner::built greedy tours" << endl;
	prettyPrint2D(tspPoseTours);

	vector< vector<int> > bestTours = tspPoseTours;
	float bestMaxLength = -1;
	for(size_t i=0; i<bestTours.size(); i++){
		float l = getTourLength(bestTours[i], poseGraph.nodeTransitions);
		if(l > bestMaxLength){ // want the smallest max length
			bestMaxLength = l;
		}
	}

	cout << "bestMaxLength: " << bestMaxLength << endl;

	vector< vector<int> > workingTours = bestTours;
	float workingMaxLength = bestMaxLength;

	cout << "GraphCoordination::tspPoseFullTourPlanner::into sim annealing" << endl;
	float temperature = 50;
	nPullsTSP = 100;
	for(int i=0; i<nPullsTSP; i++){
		cout << "GraphCoordination::tspPoseFullTourPlanner::pulling tsp" << endl;
		cout << "workingTours: " << endl;
		prettyPrint2D(workingTours);

		cout << "GraphCoordination::tspPoseFullTourPlanner::into tspEvolveTours" << endl;
		vector<vector<int> > tempTours = tspEvolveTours(workingTours, poseGraph.nodeTransitions);
		cerr << "GraphCoordination::tspPoseFullTourPlanner::out of tspEvolveTours" << endl;

		float tempMaxLength = -1;
		for(size_t i=0; i<bestTours.size(); i++){ // get worst length tour of all agents
			float l = getTourLength(bestTours[i], poseGraph.nodeTransitions);
			if(l > tempMaxLength){
				tempMaxLength = l;
			}
		}

		/*
		cout << "GraphCoordination::tspPoseFullTourPlanner::tempMaxLength: " << tempMaxLength << endl;
		cout << "GraphCoordination::tspPoseFullTourPlanner::workingMaxLength: " << workingMaxLength << endl;
		cout << "GraphCoordination::tspPoseFullTourPlanner::bestMaxLength: " << bestMaxLength << endl;
		*/

		if( exp( (workingMaxLength - tempMaxLength)/temperature ) > (rand() % 1000)/1000 ){
			//cout << "GraphCoordination::tspPoseFullTourPlanner::new workingMaxLength: " << workingMaxLength << endl;
			workingTours = tempTours;
			workingMaxLength = tempMaxLength;
		}
		if(tempMaxLength < bestMaxLength){
			bestMaxLength = tempMaxLength;
			//cout << "GraphCoordination::tspPoseFullTourPlanner::new bestMaxLength: " << bestMaxLength << endl;
			bestTours = tempTours;
		}
		temperature = temperature * 0.995;
	}

	cout << "GraphCoordination::tspPoseFullTourPlanner::out of sim annealing" << endl;
	prettyPrint2D(bestTours);

	tspPoseTours = bestTours;
	tspPoseToursLocations.clear();

	for(size_t i=0; i<tspPoseTours.size(); i++){
		vector<Point> t;
		for(size_t j=0; j<tspPoseTours[i].size(); j++){
			t.push_back(poseGraph.nodeLocations[tspPoseTours[i][j]]);
		}
		tspPoseToursLocations.push_back(t);
	}
}

vector<int> getOpenPoses(vector<vector<int> > &tours, vector<vector<float> > &distGraph){
	vector<int> openPose;
	for(size_t d=0; d<distGraph.size(); d++){
		bool open = true;
		for(size_t t=0; t<tours.size(); t++){
			for(size_t at=0; at<tours[t].size(); at++){
				if(tours[t][at] == (int)d){
					open = false;
					break;
				}
			}
			if(!open){
				break;
			}
		}
		if(open){
			openPose.push_back(d);
		}
	}
	return openPose;
}


void buildGreedyTours(vector<vector<int> > &tours, vector<int> &oSet, vector<vector<float> > &distGraph){

	// track each agent's locations as the end of its tour

	// go through each agent's locations and select the pose it is closest to
		// check for conflict, can another agent get to that pose in less total tour length

	vector<int> goalPoses(tours.size(), -1); // holds current bids, 1 per agent
	vector<float> goalBids(tours.size(), -1); // holds current bids, 1 per agent

	while(oSet.size() > 0){ // while there are still open poses
		// add poses to agents tour that will get it searched the quickest with each agent bidding on each pose
		getBids(tours, oSet, distGraph, goalPoses, goalBids); // everyone bids on their closest pose
		marketBids(tours, oSet, goalPoses, goalBids); // bids are compared and added to tours / removed from oSet
		cout << "GraphCoordination::buildGreedyTours::oSet.size: " << oSet.size() << endl;
	}
}

void marketBids(vector<vector<int> > &tours, vector<int> &oSet, vector<int> &goalPoses, vector<float> &goalBids){
	/*
	cout << "GraphCoordination::marketBids::goalPoses at goalBids" << endl;
	for(size_t i=0; i<goalPoses.size(); i++){
		cout << goalPoses[i] << " at " << goalBids[i] << endl;
	}
	*/

	for(size_t i=0; i<goalPoses.size(); i++){  // one goal pose per agent, check to see if there is a conflict
		if(goalPoses[i] >= 0 && goalBids[i] != INFINITY){
			bool conflict = false;
			for(size_t j=0; j<goalPoses.size(); j++){ // check against each other agent for conflict
				if(goalPoses[i] == goalPoses[j] && i != j){ // is there a conflict
					conflict = true;
					if(goalBids[i] > goalBids[j]){ // did they outbid me?
						goalPoses[i] = -1; // i dont bid this round
						goalBids[i] = INFINITY; // i dont bid this round
						break;
					}
					else if(goalBids[i] == goalBids[j] && i < j){ // tied and they outrank me
						goalPoses[i] = -1; // I dont bid this round
						goalBids[i] = INFINITY; // I dont bid this round
						break;
					}
					else{ // doesn't matter there is a conflict
						goalPoses[j] = -1; // they don't bid this round
						goalBids[j] = INFINITY; // they don't bid this round
						conflict = false;
					}
				}
			}
			if( !conflict ){ // conflict and I lost
				//cout << "no conflict on " << goalPoses[i] << " with index " << i << endl;
				//cout << "GraphCoordination::marketBids::oSet.size(): " << oSet.size() << endl;
				for(size_t j=0; j<oSet.size(); j++){
					if(oSet[j] == goalPoses[i]){
						oSet.erase(oSet.begin()+j); // take off the open list
						break;
					}
				}
				tours[i].push_back(goalPoses[i]); // add to my tour
			}
		}
	}
}


void getBids(vector< vector<int> > &tours, vector<int> &oSet, vector<vector<float> > distGraph, vector<int> &goalPoses, vector<float> &goalBids){
	for(size_t i=0; i<tours.size(); i++){ // for all agents get closest open pose
		float minDist = INFINITY;
		int mindex = -1;
		for(size_t j=0; j<oSet.size(); j++){
			if(distGraph[ tours[i].back() ][ oSet[j] ] < minDist){ // from the end of my tour to all open poses
				minDist = distGraph[ tours[i].back() ][ oSet[j] ];
				mindex = oSet[j];
			}
		}
		goalPoses[i] = mindex;
		goalBids[i] = getTourLength(tours[i], distGraph) + minDist;
	}
}

float getTourLength(vector<int> &tour, vector<vector<float> > &distGraph){
	float length = 0;

	if(tour.size() < 2){
		return 0;
	}
	else{
		for(size_t i=1; i<tour.size(); i++){
			length += distGraph[tour[i]][tour[i-1]];
		}
		return length;
	}
}

vector<vector<int> > tspEvolveTours(vector<vector<int> > &tours, vector<vector<float> > &distGraph){

	vector<vector<int> > tempTours = tours;
	cout << "GraphCoordination::tspEvolveTours::tours.size(): " << tempTours.size() << endl;
	prettyPrint2D(tempTours);


	int maxPulls = 1;
	for(int pull=0; pull<maxPulls; pull++){

	int c = rand() % 1000;
	if(c < 400){
		swapPosesBetweenAgents(tempTours); // exchange, each give / take 1:n poses
		cout << "GraphCoordination::tspEvolveTours::tours.size() after swap poses: " << tempTours.size() << endl;
		prettyPrint2D(tempTours);
	}
	else if(c < 800){
		transferPoses(tempTours); // don't exchange, remove and give 1:n poses
		cout << "GraphCoordination::tspEvolveTours::tours.size() after transfer: " << tempTours.size() << endl;
		prettyPrint2D(tempTours);
	}
	else{
		swapEntireTours(tempTours);
		cout << "GraphCoordination::tspEvolveTours::tours.size(): after swap entire" << tempTours.size() << endl;
		prettyPrint2D(tempTours);
	}

	// evolveIndividualTours(tempTours, distGraph);

	// while evolving individual paths, maybe find nodes that add the largest cost and place
	//		them on the bench with added cost?
	// then other agents can pull from bench if their cost is less?

	}
	cerr << "A " << endl;

	return tempTours;
}

void swapPosesBetweenAgents(vector<vector<int> > &tours){

	int a1,a2;

	// should I check that a1 and a2 are viable for swap?
	a1 = rand() % tours.size();
	a2 = a1;
	while(a1 == a2){
		a2 = rand() % tours.size();
	}

	if(tours[a1].size() > 1 && tours[a2].size() > 1){ // both have at least 1

		int nSwaps = 1;

		if(rand() % 1000 > 500 && tours[a1].size() >= 2 && tours[a2].size() >= 2){
			nSwaps = 2;
		}

		for(int i=0; i<nSwaps; i++){
			int p1 = rand() % (tours[a1].size()-1) + 1;
			int p2 = rand() % (tours[a2].size()-1) + 1;
			int t = tours[a1][p1];
			tours[a1][p1] = tours[a2][p2];
			tours[a2][p2] = t;
		}
	}
}

void transferPoses(vector<vector<int> > &tours){

	int a1,a2;
	// should I check that a1 and a2 are viable for swap?
	a1 = rand() % tours.size();
	a2 = a1;
	while(a1 == a2){
		a2 = rand() % tours.size();
	}

	if(tours[a1].size() > 1 || tours[a2].size() > 1){ // one has at least 2
		float p = 1000 * (tours[a1].size()-1) / (tours[a1].size() + tours[a2].size() -2);

		int donor = a1;
		int recip = a2;
		if(rand() % 1000 > p){ // larger share you have the more likely you give
			donor = a2;
			recip = a1;
		}

		int t = rand() % (tours[donor].size()-1) + 1;

		tours[recip].push_back( tours[donor][t] );
		tours[donor].erase(tours[donor].begin() + t);
	}
}

void swapEntireTours(vector<vector<int> > &tours){

	int a1,a2;
	// should I check that a1 and a2 are viable for swap?
	a1 = rand() % tours.size();
	a2 = a1;
	while(a1 == a2){
		a2 = rand() % tours.size();
	}

	vector<int> t;
	for(size_t i=1; i<tours[a1].size(); i++){
		t.push_back( tours[a1][i] );
	}
	int t1 = tours[a1][0];
	tours[a1].clear();
	tours[a1].push_back(t1);

	for(size_t i=1; i<tours[a2].size(); i++){
		tours[a1].push_back(tours[a2][i]);
	}
	int t2 = tours[a2][0];

	tours[a2].clear();
	tours[a2].push_back(t2);
	for(size_t i=0; i<t.size(); i++){
		tours[a2].push_back(t[i]);
	}
}

void evolveIndividualTours(vector<vector<int> > &tours, vector<vector<float> > &distGraph){

	for(size_t i=0; i<tours.size(); i++){
		cout << "GraphCoordination::evolveIndividualTours::tours[" << i << "].size(): " << tours[i].size() << endl;
		if(tours[i].size() < 8){
			cout << "GraphCoordination::evolveIndividualTours::into brute force individual solver" << endl;
			cout << "GraphCoordination::evolveIndividualTours::tours[" << i << "].size(): " << tours[i].size() << endl;
			//bruteForceTSP( tours[i], distGraph );
			vector<int> temp;
			temp.push_back(tours[i][0]);
			float tempLength = 0;

			vector<int> oSet;
			for(size_t j=1; j<tours[i].size(); j++){
				oSet.push_back(tours[i][j]);
			}
			vector<int> minTour;
			float minLength = INFINITY;

			cout << "GraphCoordination::evolveIndividualTours::in tspAddNode" << endl;
			tspRecursiveSolver(temp, tempLength, distGraph, oSet, tours[i], minLength);
			cout << "GraphCoordination::evolveIndividualTours::out of tspAddNode with length: " << minLength << endl;
		}
		else{ // too many nodes for brute force, use simulated annealing

			float temp = 1000;
			int nPullsTSP = 5;
			vector<int> bestTour = tours[i];
			vector<int> workingTour = tours[i];
			float bestLength = getTourLength( workingTour, distGraph );
			float workingLength = bestLength;


			for(int iter=0; iter<nPullsTSP; iter++){
				vector<int> tempTour = workingTour;
				evoluAlgTSP(tempTour, distGraph);
				float tempLength = getTourLength( tempTour, distGraph );

				if( exp( (workingLength - tempLength)/temp ) > (rand() % 1000)/1000 ){
					workingTour = tempTour;
					workingLength = tempLength;
				}

				if(tempLength < bestLength){
					bestLength = tempLength;
					bestTour = tempTour;
				}

				temp = temp * 0.9999;
			}
		}
	}
}


void tspRecursiveSolver( vector<int> tour, float tourLength, vector<vector<float> > &distGraph, vector<int> oSet, vector<int> &minTour, float &minLength){

	if(oSet.size() == 0){ // is the search complete
		if(tourLength < minLength){
			minLength = tourLength;
			minTour = tour;
		}
	}
	else{ // search incomplete

		for(size_t i=0; i<oSet.size(); i++){ // try all children of this node

			float di = distGraph[tour.back()][oSet[0]] + tourLength; // get distance

			if( di < minLength){ // do NOT prune, still viable

				tour.push_back(oSet[0]); // add to tour
				tourLength += di; // add to length
				oSet.erase( oSet.begin() ); // erase from oSet

				tspRecursiveSolver(tour, tourLength, distGraph, oSet, minTour, minLength); // recursive call

				// undo all changes to path from previous branch
				tourLength -= di;
				oSet.push_back( tour.back() );
				tour.pop_back();
			}
			else{
			//	cout << "GraphCoordination::tspAddNode::branch not viable" << endl;
				int t = oSet[0];
				oSet.erase( oSet.begin() );
				oSet.push_back(t);
			}
		}
	}
}

void evoluAlgTSP(vector<int> &tour, vector<vector<float> > &distGraph){
	if(tour.size() > 2){
		int nSwaps = 1;
		if(rand() % 1000 > 500){ // swap 1
			nSwaps = 2;
		}

		for(int i=0; i<nSwaps; i++){
			int s1 = rand() % (tour.size()-1) + 1;
			int s2 = s1;
			while(s1 == s2){
				s2 = rand() % (tour.size()-1) + 1;
			}
			int t = tour[s1];
			tour[s1] = tour[s2];
			tour[s2] = t;
		}
	}
}

float GraphCoordination::getGraphObservations(Graph &graph, Costmap &costmap, Mat &gView, vector<int> &workingSet){
	graph.nodeObservations.clear();
	for(size_t i=0; i<graph.nodeLocations.size(); i++){
		Mat t = Mat::zeros(costmap.cells.size(), CV_8UC1);
		graph.nodeObservations.push_back(t);
	}
	for(size_t i=0; i<workingSet.size(); i++){ // simulate observations from all nodes
		simulateObservation(graph.nodeLocations[workingSet[i] ], graph.nodeObservations[workingSet[i] ], costmap);
		bitwise_or(gView, graph.nodeObservations[workingSet[i] ], gView);
	}
	return matReward(gView);
}

vector<int> GraphCoordination::getWorkingSet(Graph &graph, Costmap &costmap){
	vector<int> workingSet;
	for(size_t i = 0; i<graph.nodeLocations.size(); i++){
		float d = graph.aStarDist(i, graph.cNode, costmap);
		if( d < INFINITY && graph.nodeLocations[i] != graph.nodeLocations[graph.cNode]){ // reachable and not current node
			workingSet.push_back(i);
		}
	}
	return workingSet;
}


void GraphCoordination::getOset(Graph &graph, Costmap &costmap, vector<int> &oStates, vector<int> &cStates, vector<int> &workingSet){
	for(size_t i = 0; i<workingSet.size(); i++){
		bool flag = true;
		for(size_t j=0; j<tempGraph.nodeLocations.size(); j++){
			if(tempGraph.nodeLocations[j] == graph.nodeLocations[workingSet[i] ]){
				cStates.push_back(workingSet[i]);
				flag = false;
				break;
			}
		}
		if(flag){
			oStates.push_back(workingSet[i]);
		}
	}
}

float GraphCoordination::getCurrentPoseSetReward(Graph &graph, Costmap &costmap, Mat &cView, vector<int> &workingSet){
	// check if poseGraph nodes are still on graph, if yes import their view, else erase

	for(size_t j=0; j<tempGraph.nodeLocations.size(); j++){
		bool flag = true;
		for(size_t i = 0; i<workingSet.size(); i++){
			if(tempGraph.nodeLocations[j] == graph.nodeLocations[workingSet[i] ]){
				tempGraph.nodeObservations[j] = graph.nodeObservations[workingSet[i] ];

				bitwise_or(cView, graph.nodeObservations[workingSet[i] ], cView);
				flag = false;
				break;
			}
		}
		if(flag){
			tempGraph.nodeLocations.erase(tempGraph.nodeLocations.begin()+j);
			tempGraph.nodeObservations.erase(tempGraph.nodeObservations.begin()+j);
		}
	}
	return matReward(cView);
}

int GraphCoordination::matReward(Mat &in){
	// get Mat entropy
	float observed = 0;
	for(int i=0; i<in.rows; i++){
		for(int j=0; j<in.cols; j++){
			if(in.at<uchar>(i,j,0) > 0){
				observed++;
			}
		}
	}
	return observed;
}

void GraphCoordination::displayPoseGraph(Costmap &costmap){
	Mat view = Mat::zeros(costmap.cells.size(), CV_8UC1);
	for(size_t i=1; i<poseGraph.nodeLocations.size(); i++){
		bitwise_or(view, poseGraph.nodeObservations[i], view);
	}
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		circle(view, poseGraph.nodeLocations[i], 2, Scalar(127), -1);
		char buff[5];
		sprintf(buff, "%i", i);
		putText(view, buff, poseGraph.nodeLocations[i], 0, 0.35, Scalar(127));
	}

	circle(view, poseGraph.nodeLocations[0], 3, Scalar(255), -1);
	char buff[5];
	sprintf(buff, "C");
	putText(view, buff, poseGraph.nodeLocations[0], 0, 0.35, Scalar(127));

	namedWindow("GraphCoordination::poseGraph.nodes.observations", WINDOW_NORMAL);
	imshow("GraphCoordination::poseGraph.nodes.observations", view);
}

void GraphCoordination::displayPoseTours(Costmap &costmap){
	// build plot to show off the path
	Mat tMat = Mat::zeros(costmap.cells.size(), CV_8UC1);;
	for(size_t i=1; i<poseGraph.nodeLocations.size(); i++){
		bitwise_or(tMat, poseGraph.nodeObservations[i], tMat);
	}

	for(size_t j=0; j<tspPoseTours.size(); j++){

		cout << "tspPoseTours[" << tspPoseTours[j].size() << "]: " << poseGraph.nodeLocations[tspPoseTours[j][0]].x << " / " << poseGraph.nodeLocations[tspPoseTours[j][0]].y;
		for(size_t i=1; i<tspPoseTours[j].size(); i++){
			cout << ", " << poseGraph.nodeLocations[tspPoseTours[j][i]].x << " / " << poseGraph.nodeLocations[tspPoseTours[j][i]].y;
			line(tMat, poseGraph.nodeLocations[tspPoseTours[j][i]],  poseGraph.nodeLocations[tspPoseTours[j][i-1]], Scalar(127), 1);
		}
		cout << endl;

		for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
			circle(tMat, poseGraph.nodeLocations[i], 2, Scalar(127), -1);
		}
		circle(tMat, poseGraph.nodeLocations[tspPoseTours[j][1]], 2, Scalar(200), 3);
	}
	namedWindow("ws view", WINDOW_NORMAL);
	imshow("ws view", tMat);
}

void GraphCoordination::findPoseGraphTransitions(Graph &graph, Costmap &costmap){
	poseGraph.nodeTransitions.clear();
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		vector<float> t;
		for(size_t j=0; j<poseGraph.nodeLocations.size(); j++){
			t.push_back(INFINITY);
		}
		poseGraph.nodeTransitions.push_back(t);
	}

	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		for(size_t j=0; j<poseGraph.nodeLocations.size(); j++){
			if(i != j){
				float d = costmap.aStarDist(poseGraph.nodeLocations[i],poseGraph.nodeLocations[j]);
				poseGraph.nodeTransitions[i][j] = d;
				poseGraph.nodeTransitions[j][i] = d;
			}
		}
	}

	/*
	cout << "GraphCoordination::findPoseGraphTransitions::nodeTransitions array:" << endl;
	for(size_t i=0; i<poseGraph.nodeTransitions.size(); i++){
		cout << "   ";
		for(size_t j=0; j<poseGraph.nodeTransitions.size(); j++){
			cout <<poseGraph.nodeTransitions[i][j] << ", ";
		}
		cout << endl;
	}
	*/
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

