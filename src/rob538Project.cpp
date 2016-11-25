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

int main(){

	destroyAllWindows();

	vector<int> treePath;
	srand( time(NULL) );
	bool videoFlag = false;

	int numAgents = 4;
	int numIterations = 1;

	int gSpace = 2;
	float obsThresh = 40;
	float comThresh = 80;
	int maxTime = 100;

	vector<string> fName;
	//fName.push_back("mineMap");
	fName.push_back("test6");
	//fName.push_back("openTest");
	//fName.push_back("slamMap");
	//fName.push_back("tunnelTest");

	int map_iter = 0;
	cout << "fName.size(): " << fName.size() << " & mapIter: " << map_iter << endl;

	string mapLog;
	vector<int> timeLog;
	vector<float> percentObservedLog;

	srand( time(NULL) );
	// create world
	World world(fName[map_iter], gSpace, obsThresh, comThresh);
	cout << "main::loaded world" << fName[ map_iter ] << endl;

	vector<Point> sLoc;
	for(int i=0; i<numIterations*numAgents; i++){
		while(true){
			Point tLoc(rand() % world.costmap.cells.cols, rand() % world.costmap.cells.rows);
			if(world.costmap.cells.at<short>(tLoc) == world.costmap.obsFree){ // its not an obstacle
				sLoc.push_back(tLoc);
				break;
			}
		}
	}
	cout << "main::Chose starting locations" << endl;

	for(int iterations_iter = 0; iterations_iter<numIterations; iterations_iter++){

		Observer humanObserver(sLoc[iterations_iter], numAgents, false, "operator");
		Observer globalObserver(sLoc[iterations_iter], numAgents, true, "global");// make this humanObserver get maps shared with it and NOT share its map with people it

		vector<Agent> agents;
		for(int i=0; i<numAgents; i++){

			vector<float> constants;
			for(int j=0; j<5; j++){
				constants.push_back(1);//float(rand() % 1000) / 100 - 50);
			}

			if(i < 2){ // make a relay
				constants.clear();
				constants.push_back(-1); // frontier cells
				constants.push_back(1000); // frontier travel cost
				constants.push_back(0); // dComArea as relay
				constants.push_back(1); // dRelays as relay
				constants.push_back(1); // dExplorers as relay
				constants.push_back(1); // relay travel cost
			}
			else{ // make an explorer
				constants.clear();
				constants.push_back(100); // frontier cells
				constants.push_back(1); // frontier travel cost
				constants.push_back(-10); // dComArea as relay
				constants.push_back(0); // dRelays as relay
				constants.push_back(0); // dExplorers as relay
				constants.push_back(1); // relay travel cost
			}

			if( false ){ // different starting location?
				agents.push_back(Agent(sLoc[iterations_iter*numAgents], i, world, obsThresh, comThresh, numAgents, constants));
				cout << "Main::Agent[" << i << "]: created at : " << sLoc[iterations_iter*numAgents+i].x << " , " << sLoc[iterations_iter*numAgents+i].y << endl;
			}
			else{
				agents.push_back(Agent(sLoc[iterations_iter*numAgents], i, world, obsThresh, comThresh, numAgents, constants));
				cout << "Main::Agent[" << i << "]: created at : " << sLoc[iterations_iter*numAgents].x << " , " << sLoc[iterations_iter*numAgents].y << endl;
			}
		}
		cout << "Main::All agents created" << endl;

		time_t start = clock();

		// video writers
		VideoWriter slamVideo;
		if(videoFlag){
			//Size frameSize( static_cast<int>(world.costmap.cells.cols, world.costmap.cells.rows) );
			//slamVideo.open("multiAgentInferen ce.avi",CV_FOURCC('M','J','P','G'), 30, frameSize, true );
			cout << "Main::Videos started" << endl;
		}

		cout << "Main::Ready, press any key to begin." << endl;
		waitKey(1);
		cout << "Main::Here we go!" << endl;

		world.observe(humanObserver.cLoc, humanObserver.costmap);

		int timeSteps = -1;
		float percentObserved = 0;
		while(timeSteps < maxTime-1 && percentObserved < 0.99){
			cout << "Main::Starting while loop" << endl;
			timeSteps++;
			waitKey(1);

			// all agents observe
			for(int i=0; i<numAgents; i++){
				world.observe(agents[i].cLoc, agents[i].costmap);
				agents[i].market.updateMarket(agents[i].cLoc, agents[i].gLoc);

				world.observe(agents[i].cLoc, globalObserver.costmap);
				globalObserver.market.cLocs[agents[i].myIndex] = agents[i].cLoc;
				globalObserver.market.gLocs[agents[i].myIndex] = agents[i].gLoc;
			}

			cout << "Main::made observations" << endl;

			for(int commHops = 0; commHops < numAgents; commHops++){
				// all agents communicate
				for(int i=0; i<numAgents; i++){
					// all agents communicate with global observer, always
					agents[i].communicate( globalObserver.costmap, globalObserver.market );
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
			cout << "Main::Out of communicate with other agents" << endl;


			humanObserver.showCellsPlot();
			waitKey(1);
			globalObserver.showCellsPlot();
			waitKey(1);
			humanObserver.market.iterateTime();

			cout << "Main::Into plan for one timestep" << endl;
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
			cout << "------timeSteps & percent observed: " << timeSteps << " & " << percentObserved << " " << map_iter << ": " << fName[map_iter] << endl;

		} // end timeStep in simulation

		cerr << "made it to the end of the simulation!" << endl;

		vector<float> dReward = getDifferenceRewards( agents, world, humanObserver.cLoc, timeSteps, percentObserved);

		cout << "dReward: ";
		for(size_t i=0; i<dReward.size(); i++){
			cout << dReward[i] << ", ";
		}
		cout << endl;


	} // end iterations
	ofstream myfile;
	myfile.open ("/home/andy/git/rob538Project/results/run1.csv", ofstream::out | ofstream::app);
	for(size_t i=0; i<timeLog.size(); i++){
		myfile << mapLog << ",";
		myfile << timeLog[i] << ",";
		myfile << percentObservedLog[i] << ",";
	}
	myfile.close();

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
					agents[a].cLoc = agents[a].history[t];
				}
			}

			// all agents observe
			for(size_t a=0; a<agents.size(); a++){
				if(a != na){ // exclude diff agent
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
