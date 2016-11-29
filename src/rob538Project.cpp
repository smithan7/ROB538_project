//============================================================================
// Name        : rob538Project.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include<dirent.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Agent.h"
#include "Observer.h"

using namespace cv;
using namespace std;

float getPercentObserved(Costmap &globalCostmap, Costmap &workingCostmap);
vector<float> getDifferenceRewards( vector<Agent> &agents, World &world, Point oLoc, int timeSteps, float gReward);
float runTest(vector<vector<float> > constants, Point sLoc, World world, int numAgents, int maxTime);
vector<float> mutate(vector<float> a);

int main(){

	srand(time(NULL));
	int numAgents = 3;
	int popSize = 6;
	vector<vector<float> > constants;

	for(int i=0; i<popSize; i++){
		vector<float> t;
		for(int j=0; j<5; j++){
			t.push_back(float(rand() % 10000) / 100 - 50); // rand -50 -> 50 // frontier cells, frontier travel cost, dComArea, dRealys, dExplorers, relay travel cost
		}
		constants.push_back( t );
	}

	int gSpace = 4;
	float obsThresh = 30;
	float comThresh = 60;
	int maxTime = 100;

	//string fName = "test6";
	//fName.push_back("mineMap");
	//fName.push_back("test6");
	//fName.push_back("openTest");
	string fName = "slamMap";
	//fName.push_back("tunnelTest");

	// create world
	World world(fName, gSpace, obsThresh, comThresh);
	//cout << "main::loaded world" << fName << endl;

	int maxGen = 50;
	for(int generations = 0; generations < maxGen; generations++){ // how many generations

		Point sLoc;
		while(true){
			Point tLoc(rand() % world.costmap.cells.cols, rand() % world.costmap.cells.rows);
			if(world.costmap.cells.at<short>(tLoc) == world.costmap.obsFree){ // its not an obstacle
				sLoc = tLoc;
				break;
			}
		}

		vector<int> indexList;
		vector<float> scoreList;
		for(int i=0; i<popSize; i++){
			indexList.push_back(-1);
			scoreList.push_back(-INFINITY);
		}

		for(int pop = 0; pop < popSize; pop++){ // for each member of the populatioin

			cerr << "constants.size(): " << constants.size();
			cerr << "constants[pop].size(): " << constants[pop].size() << endl;
			vector<vector<float> > testConsts;
			testConsts.push_back(constants[pop]);

			float mScoreA = 0;
			float mScoreB = 0;

			for( int pIter = 0; pIter < 1; pIter++){ // how many iterations to test each agent

				for(int a=1; a<numAgents; a++){ // get the other agents
					int ta = rand() % popSize;
					testConsts.push_back( constants[ ta ] );
				}

				float score = runTest( testConsts, sLoc, world, numAgents, maxTime); // run test
				int index = pop;

				for(int i=0; i<popSize; i++){
					if( score > scoreList[i]){
						float ts = scoreList[i];
						float ti = indexList[i];

						scoreList[i] = score;
						indexList[i] = index;

						score = ts;
						index = ti;
					}
				}

			}
		}


		cerr << "into training" << endl;
		vector<vector<float> > tConst;
		for(int i=0; i<popSize/2; i++){
			tConst.push_back( constants[ indexList[i]]);
			tConst.push_back( mutate( constants[ indexList[i] ]) );
		}
		cerr << "out of training" << endl;
		constants = tConst;
		cerr << "saved training" << endl;
	}
}

vector<float> mutate(vector<float> a){

	for(int i=0; i<6; i++){
		a[i] = a[i] + float(rand() % 1000 / 100) - 5;
	}
	return a;
}

float runTest(vector<vector<float> > constants, Point sLoc, World world, int numAgents, int maxTime){

	float obsThresh = world.obsThresh;
	float comThresh = world.commThresh;


	string mapLog;
	vector<int> timeLog;
	vector<float> percentObservedLog;

	srand( time(NULL) );

	vector<Agent> agents;
	for(int i=0; i<numAgents; i++){
		agents.push_back( Agent(sLoc, i, world, obsThresh, comThresh, numAgents, constants[i]) );
	}

	Observer humanObserver(sLoc, numAgents, false, "operator");
	//Observer globalObserver(sLoc, numAgents, true, "global");// make this humanObserver get maps shared with it and NOT share its map with people it

	time_t start = clock();

	// video writers
	VideoWriter slamVideo;
	bool videoFlag = false;
	if(videoFlag){
		//Size frameSize( static_cast<int>(world.costmap.cells.cols, world.costmap.cells.rows) );
		//slamVideo.open("multiAgentInferen ce.avi",CV_FOURCC('M','J','P','G'), 30, frameSize, true );
		//cout << "Main::Videos started" << endl;
	}

	//cout << "Main::Ready, press any key to begin." << endl;
	waitKey(1);
	//cout << "Main::Here we go!" << endl;

	world.observe(humanObserver.cLoc, humanObserver.costmap);

	int timeSteps = -1;
	float percentObserved = 0;
	while(timeSteps < maxTime-1 && percentObserved < 0.99){
		//cout << "Main::Starting while loop" << endl;
		timeSteps++;
		waitKey(1);

		// all agents observe
		for(int i=0; i<numAgents; i++){
			if( agents[i].market.roles[i] == 'e'){
				world.observe(agents[i].cLoc, agents[i].costmap);
				agents[i].market.updateMarket(agents[i].cLoc, agents[i].gLoc);

				//world.observe(agents[i].cLoc, globalObserver.costmap);
				//globalObserver.market.cLocs[agents[i].myIndex] = agents[i].cLoc;
				//globalObserver.market.gLocs[agents[i].myIndex] = agents[i].gLoc;
			}
		}

		//cout << "Main::made observations" << endl;

		for(int commHops = 0; commHops < numAgents; commHops++){
			// all agents communicate
			for(int i=0; i<numAgents; i++){
				// all agents communicate with global observer, always
				//agents[i].communicate( globalObserver.costmap, globalObserver.market );
				// all agents communicate with humanObserver, if possible
				if( world.commoCheck(agents[i].cLoc, humanObserver.cLoc, comThresh) ){
					agents[i].communicate( humanObserver.costmap, humanObserver.market );
					humanObserver.communicate( agents[i].costmap, agents[i].market );
					agents[i].market.contactWithObserver = true;
				}
				// all agents communicate with each other if possible
				for(int j=0; j<numAgents; j++){
					if(i!=j){
						if(world.commoCheck(agents[i].cLoc, agents[j].cLoc, comThresh)){
							agents[i].communicate( agents[j].costmap, agents[j].market );
							agents[j].communicate( agents[i].costmap, agents[i].market );
						}
					}
				}
			}
		}
		//cout << "Main::Out of communicate with other agents" << endl;


		humanObserver.showCellsPlot();
		waitKey(1);
		//globalObserver.showCellsPlot();
		//waitKey(1);
		humanObserver.market.iterateTime();

		//cout << "Main::Into plan for one timestep" << endl;
		// all agents plan for one timestep
		for(int i=0; i<numAgents; i++){
			agents[i].planRoleSwapping(); // includes updating internal cLoc
		}

		// all agents act for one timestep
		for(int i=0; i<numAgents; i++){
			agents[i].act();
		}

		if(videoFlag){
			for(int i =0; i<5; i++){ // slower frmae rate through repeated frames
				slamVideo << humanObserver.costmap.displayPlot;
			}
		}

		// print out progress
		percentObserved = getPercentObserved(world.costmap, humanObserver.costmap);
		//cout << "------timeSteps & percent observed: " << timeSteps << " & " << percentObserved << endl;

	} // end timeStep in simulation
	//cerr << ", " << percentObserved << ", ";

	//cout << "main::made it to the end of the simulation!" << endl;

	vector<float> dReward = getDifferenceRewards( agents, world, humanObserver.cLoc, timeSteps, percentObserved);

	cout << "dReward: ";
	for(size_t i=0; i<dReward.size(); i++){
		cout << dReward[i] << ", ";
	}
	cout << endl;
	return dReward[0];


	return percentObserved;

	/*
	ofstream myfile;
	myfile.open ("/home/andy/git/rob538Project/results/run1.csv", ofstream::out | ofstream::app);
	for(size_t i=0; i<timeLog.size(); i++){
		myfile << mapLog << ",";
		myfile << timeLog[i] << ",";
		myfile << percentObservedLog[i] << ",";
	}
	myfile.close();
	*/

	cerr << "wrote the file!" << endl;
	waitKey(0);
} // end maps


vector<float> getDifferenceRewards( vector<Agent> &agents, World &world, Point oLoc, int timeSteps, float gReward){

	vector<float> dRewards;

	for(size_t na=0; na<agents.size(); na++){

		Observer tO(oLoc, agents.size(), false, "diff reward");
		for(size_t a=0; a<agents.size(); a++){
			agents[a].costmap.cells.release();
		}

		for(int t=0; t<timeSteps; t++){

			// get position from timestep
			for(size_t a=0; a<agents.size(); a++){
				if( a != na ){
					agents[a].cLoc = agents[a].pathHistory[t];
				}
			}

			// all agents observe
			for(size_t a=0; a<agents.size(); a++){
				if(a != na && agents[a].roleHistory[t] == 'e' || t == 0 ){ // exclude diff agent
					world.observe(agents[a].cLoc, agents[a].costmap);
					agents[a].market.updateMarket(agents[a].cLoc, agents[a].gLoc);
				}
			}
			world.observe(tO.cLoc, tO.costmap);

			// all agents communicate
			for(size_t hops = 0; hops < agents.size(); hops++){
				for(size_t a=0; a<agents.size(); a++){
					// all agents communicate with humanObserver, if possible
					if(a != na){ // exclude diff agent
						if(world.commoCheck(agents[a].cLoc, tO.cLoc, agents[a].comThresh)){
							agents[a].communicate(tO.costmap, tO.market);
							tO.communicate(agents[a].costmap, agents[a].market);
						}
						for(size_t b=a+1; b<agents.size(); b++){
							if(a!=b && b!=na){
								if(world.commoCheck(agents[a].cLoc, agents[b].cLoc, agents[a].comThresh)){
									agents[a].communicate(agents[b].costmap, agents[b].market);
									agents[b].communicate(agents[a].costmap, agents[a].market);
								}
							}
						}
					}
				}
			} // end commo

			tO.showCellsPlot();
			waitKey(1);
			tO.market.iterateTime();

		} // end 1 sim iter

		dRewards.push_back( gReward - getPercentObserved( world.costmap, tO.costmap));
	}

	return dRewards;
}


float getPercentObserved(Costmap &globalCostmap, Costmap &workingCostmap){

	float globalFree = 0;
	float workingFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}
			if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				workingFree++;
			}
		}
	}

	return workingFree / globalFree;
}
