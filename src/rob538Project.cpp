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
#include "Population.h"

using namespace cv;
using namespace std;

float getPercentObserved(Costmap &globalCostmap, Costmap &workingCostmap);
vector<float> getDifferenceRewards( vector<Agent> &agents, World &world, Point oLoc, int timeSteps, float gReward);
vector<float> runTest(vector<vector<float> > constants, Point sLoc, World world, int numAgents, int maxTime, bool differenceRewards);
float getMean( vector<float> g);
float getStdDev(float mean, vector<float> g);

int main(){

	//maybe evaluate each node for exploration too? combine to one function?

	srand(time(NULL));
	int numAgents = 7;
	int popSize = 4;
	int numLenIters = 2;
	int nConstants = 6;

	vector<Population> pops, testPopulations;
	for(int i=0; i<numAgents; i++){
		pops.push_back( Population( popSize, nConstants) );
	}
	testPopulations = pops;

	int gSpace = 4;
	float obsThresh = 25;
	float comThresh = 38;
	int maxTime = 100;
	bool differenceRewards, leniency, hallOfFame;

	//string fName = "test6";
	//fName.push_back("mineMap");
	//fName.push_back("test6");
	//fName.push_back("openTest");
	string fName = "slamMap";
	//fName.push_back("tunnelTest");


	// create world
	World world(fName, gSpace, obsThresh, comThresh);
	//cout << "main::loaded world" << fName << endl;

	int maxGen = 25;

	vector<float> gGen;

	/*
	Point sLoc;
	while(true){
		Point tLoc(rand() % world.costmap.cells.cols, rand() % world.costmap.cells.rows);
		if(world.costmap.cells.at<short>(tLoc) == world.costmap.obsFree){ // its not an obstacle
			sLoc = tLoc;
			break;
		}
	}
	cerr << "sLoc: " << sLoc << endl;
	*/
	//Point sLoc(60,90);
	Point sLoc(15,15);

	for(int bigTestIter = 0; bigTestIter < 6; bigTestIter++){

		if( bigTestIter == 0 ){ // global reward or difference rewards
			cerr << "global: ";
			pops = testPopulations; // initialize
			for(int generations = 0; generations < maxGen; generations++){ // how many generations
				for(int i = 0; i<numAgents; i++){
					pops[i].learningRate *= 0.5;
				}
				vector<float> g; // tracks global reward

					for(int i=0; i<numAgents; i++){
						pops[i].reset();
					}

					for(int pIter = 0; pIter < popSize; pIter++){ // for each member of the population

						vector<vector<float> > testConsts; // numAgents in test x 6

						for(int a=0; a<numAgents; a++){ // get random other agents
							int choose =  rand() % pops[a].open.size() ;
							pops[a].testIndex = pops[a].open[choose]; // rand agent
							pops[a].open.erase( pops[a].open.begin() + choose); // erase from available pool
							testConsts.push_back( pops[a].constants[pops[a].testIndex] );
						}

						vector<float> rewards = runTest( testConsts, sLoc, world, numAgents, maxTime, false); // run test

						for(int a = 0; a < numAgents; a++){ // for each agent, check lenient score
							pops[a].rewards[pops[a].testIndex] = rewards[0];
						}
						g.push_back( rewards[0] ); // track average global score
					}

				for(int a=0; a<numAgents; a++){ // rank scores and indices
					pops[a].mutate();
				}
				float mg = getMean(g);
				float sg = getStdDev( mg, g);
				cerr << mg << ", " << sg << ",  ";
			}
		}
		else if( bigTestIter == 1 ){ // global reward or difference rewards
			cerr << "global lenient: ";
			pops = testPopulations; // initialize
			for(int generations = 0; generations < maxGen; generations++){ // how many generations
				for(int i = 0; i<numAgents; i++){
					pops[i].learningRate *= 0.5;
				}
				vector<float> g; // tracks global reward

				for( int lIter = 0; lIter < numLenIters; lIter++){ // how many iterations to test agents

					for(int i=0; i<numAgents; i++){
						pops[i].reset();
					}

					for(int pIter = 0; pIter < popSize; pIter++){ // for each member of the population

						vector<vector<float> > testConsts; // numAgents in test x 6

						for(int a=0; a<numAgents; a++){ // get random other agents
							int choose =  rand() % pops[a].open.size() ;
							pops[a].testIndex = pops[a].open[choose]; // rand agent
							pops[a].open.erase( pops[a].open.begin() + choose); // erase from available pool
							testConsts.push_back( pops[a].constants[pops[a].testIndex] );
						}

						vector<float> rewards = runTest( testConsts, sLoc, world, numAgents, maxTime, false); // run test

						for(int a = 0; a < numAgents; a++){ // for each agent, check lenient score
							if(rewards[a] > pops[a].rewards[pops[a].testIndex]){ // find best score of each agent, lenient
								pops[a].rewards[pops[a].testIndex] = rewards[0];
							}
						}
						g.push_back( rewards[0] ); // track average global score
					}
				}
				for(int a=0; a<numAgents; a++){ // rank scores and indices
					pops[a].mutate();
				}
				float mg = getMean(g);
				float sg = getStdDev( mg, g);
				cerr << mg << ", " << sg << ",  ";
			}
		}
		else if( bigTestIter == 2 ){ // global reward or difference rewards
			cerr << "difference: ";
			pops = testPopulations; // initialize
			for(int generations = 0; generations < maxGen; generations++){ // how many generations
				for(int i = 0; i<numAgents; i++){
					pops[i].learningRate *= 0.5;
				}
				vector<float> g; // tracks global reward

					for(int i=0; i<numAgents; i++){
						pops[i].reset();
					}

					for(int pIter = 0; pIter < popSize; pIter++){ // for each member of the population

						vector<vector<float> > testConsts; // numAgents in test x 6

						for(int a=0; a<numAgents; a++){ // get random other agents
							int choose =  rand() % pops[a].open.size() ;
							pops[a].testIndex = pops[a].open[choose]; // rand agent
							pops[a].open.erase( pops[a].open.begin() + choose); // erase from available pool
							testConsts.push_back( pops[a].constants[pops[a].testIndex] );
						}

						vector<float> rewards = runTest( testConsts, sLoc, world, numAgents, maxTime, true); // run test

						for(int a = 0; a < numAgents; a++){ // for each agent, check lenient score
							pops[a].rewards[pops[a].testIndex] = rewards[a];
						}
						g.push_back( rewards[numAgents] ); // track average global score
					}

				for(int a=0; a<numAgents; a++){ // rank scores and indices
					pops[a].mutate();
				}
				float mg = getMean(g);
				float sg = getStdDev( mg, g);
				cerr << mg << ", " << sg << ",  ";
			}
		}
		else if( bigTestIter == 3 ){ // global reward or difference rewards
			cerr << "difference lenient: ";

			pops = testPopulations; // initialize
			for(int generations = 0; generations < maxGen; generations++){ // how many generations
				for(int i = 0; i<numAgents; i++){
					pops[i].learningRate *= 0.5;
				}
				vector<float> g; // tracks global reward

				for( int lIter = 0; lIter < numLenIters; lIter++){ // how many iterations to test agents

					for(int i=0; i<numAgents; i++){
						pops[i].reset();
					}

					for(int pIter = 0; pIter < popSize; pIter++){ // for each member of the population

						vector<vector<float> > testConsts; // numAgents in test x 6

						for(int a=0; a<numAgents; a++){ // get random other agents
							int choose =  rand() % pops[a].open.size() ;
							pops[a].testIndex = pops[a].open[choose]; // rand agent
							pops[a].open.erase( pops[a].open.begin() + choose); // erase from available pool
							testConsts.push_back( pops[a].constants[pops[a].testIndex] );
						}

						vector<float> rewards = runTest( testConsts, sLoc, world, numAgents, maxTime, true); // run test

						for(int a = 0; a < numAgents; a++){ // for each agent, check lenient score
							if(rewards[a] > pops[a].rewards[pops[a].testIndex]){ // find best score of each agent, lenient
								pops[a].rewards[pops[a].testIndex] = rewards[a];
							}
						}
						g.push_back( rewards[numAgents] ); // track average global score
					}
				}
				for(int a=0; a<numAgents; a++){ // rank scores and indices
					pops[a].mutate();
				}
				float mg = getMean(g);
				float sg = getStdDev( mg, g);
				cerr << mg << ", " << sg << ",  ";
			}
		}
		else if( bigTestIter == 4 ){
			cerr << "hall of fame: ";

			pops = testPopulations; // initialize
			for(int generations = 0; generations < maxGen; generations++){ // how many generations

				for(int i = 0; i<numAgents; i++){
					pops[i].learningRate *= 0.5;
				}

				vector<float> g; // tracks global reward

					for(int i=0; i<numAgents; i++){
						pops[i].getStar();
						pops[i].reset();
					}

					for( int aIter = 0; aIter < numAgents; aIter++){

						for(int pIter = 0; pIter < popSize; pIter++){ // for each member of the population

							vector<vector<float> > testConsts; // numAgents in test x 6
							testConsts.push_back( pops[aIter].constants[pIter] );

							for(int a=0; a<numAgents; a++){ // get random other agents
								if( a != aIter ){
									testConsts.push_back( pops[a].constants[pops[a].indexStar] );
								}
							}

							vector<float> rewards = runTest( testConsts, sLoc, world, numAgents, maxTime, false); // run test

							pops[aIter].rewards[pIter] = rewards[0];

							g.push_back( rewards[0] ); // track average global score

						}
					}
				for(int a=0; a<numAgents; a++){ // rank scores and indices
					pops[a].mutate();
				}
				float mg = getMean(g);
				float sg = getStdDev( mg, g);
				cerr << mg << ", " << sg << ",  ";
			}
		}
		else if( bigTestIter == 5 ){ // global reward or difference rewards
			cerr << "difference hall of fames: ";

			pops = testPopulations; // initialize
			for(int generations = 0; generations < maxGen; generations++){ // how many generations

				for(int i = 0; i<numAgents; i++){
					pops[i].learningRate *= 0.5;
				}

				vector<float> g; // tracks global reward

					for(int i=0; i<numAgents; i++){
						pops[i].getStar();
						pops[i].reset();
					}

					for( int aIter = 0; aIter < numAgents; aIter++){

						for(int pIter = 0; pIter < popSize; pIter++){ // for each member of the population

							vector<vector<float> > testConsts; // numAgents in test x 6
							testConsts.push_back( pops[aIter].constants[pIter] );

							for(int a=0; a<numAgents; a++){ // get random other agents
								if( a != aIter ){
									testConsts.push_back( pops[a].constants[pops[a].indexStar] );
								}
							}

							vector<float> rewards = runTest( testConsts, sLoc, world, numAgents, maxTime, true); // run test

							pops[aIter].rewards[pIter] = rewards[aIter];
							g.push_back( rewards[numAgents] ); // track average global score
						}
					}
				for(int a=0; a<numAgents; a++){ // rank scores and indices
					pops[a].mutate();
				}
				float mg = getMean(g);
				float sg = getStdDev( mg, g);
				cerr << mg << ", " << sg << ",  ";
			}
		}
		cerr << endl;
	}
}

		/*

		pops = testPopulations;
		for(int i = 0; i<numAgents; i++){
			pops[i].learningRate *= 0.5;
		}

		for(int generations = 0; generations < maxGen; generations++){ // how many generations

			vector<float> g; // tracks global reward

			// sets up leniency rewards
			int lenIters = 1;
			if(leniency){
				lenIters = numLenIters;
			}

			for( int lIter = 0; lIter < lenIters; lIter++){ // how many iterations to test agents

				for(int i=0; i<numAgents; i++){
					pops[i].reset();
				}

				for(int pIter = 0; pIter < popSize; pIter++){ // for each member of the population

					vector<vector<float> > testConsts; // numAgents in test x 6

					if( hallOfFame ){
						for(int a=0; a<numAgents; a++){ // get random other agents
							pops[a].testIndex = rand() % pops[a].open.size(); // rand agent
							pops[a].open.erase( pops[a].open.begin() + pops[a].testIndex); // erase from available pool
							testConsts.push_back( pops[a].constants[pops[a].testIndex] );
						}
					}
					else{
						for(int a=0; a<numAgents; a++){ // get random other agents
							pops[a].testIndex = rand() % pops[a].open.size(); // rand agent
							pops[a].open.erase( pops[a].open.begin() + pops[a].testIndex); // erase from available pool
							testConsts.push_back( pops[a].constants[pops[a].testIndex] );
						}
					}

					vector<float> rewards = runTest( testConsts, sLoc, world, numAgents, maxTime, differenceRewards); // run test

					if( differenceRewards ){ // use difference rewards?
						for(int a = 0; a < numAgents; a++){ // for each agent, check lenient score
							if(rewards[a] > pops[a].rewards[pops[a].testIndex]){ // find best score of each agent, lenient
								pops[a].rewards[pops[a].testIndex] = rewards[a];
							}
						}
						g.push_back( rewards[numAgents] ); // track average global score
					}
					else{ // use global rewards?
						for(int a = 0; a < numAgents; a++){ // for each agent, check lenient score
							if(rewards[a] > pops[a].rewards[pops[a].testIndex]){ // find best score of each agent, lenient
								pops[a].rewards[pops[a].testIndex] = rewards[0];
							}
						}
						g.push_back( rewards[0] ); // track average global score
					}

				}
			}

			for(int a=0; a<numAgents; a++){ // rank scores and indices
				pops[a].mutate();
				if( hallOfFame ){
					pops[a].getStar();
				}
			}



			float mg = getMean(g);
			float sg = getStdDev( mg, g);
			cerr << mg << ", " << sg << ",  ";
		}
		cerr << endl;
	}
}
*/

vector<float> runTest(vector<vector<float> > constants, Point sLoc, World world, int numAgents, int maxTime, bool differenceRewards){

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
			if( agents[i].market.roles[i] == 'e' || true){
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

	if( differenceRewards ){
		vector<float> dRewards = getDifferenceRewards( agents, world, humanObserver.cLoc, timeSteps, percentObserved);
		dRewards.push_back( percentObserved );
		return dRewards;
	}
	else{
		vector<float> gReward;
		gReward.push_back( percentObserved );
		return gReward;
	}



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
		world.observe(tO.cLoc, tO.costmap);
		for(size_t a=0; a<agents.size(); a++){
			agents[a].costmap.cells.release();
		}


		// all agents observe
		for(size_t a=0; a<agents.size(); a++){
			if(a != na){ // exclude diff agent
				world.observe(agents[a].cLoc, agents[a].costmap);
				agents[a].market.updateMarket(agents[a].cLoc, agents[a].gLoc);
			}
		}

		for(int t=0; t<timeSteps; t++){

			// get position from timestep
			for(size_t a=0; a<agents.size(); a++){
				if( a != na ){
					agents[a].cLoc = agents[a].pathHistory[t];
				}
			}

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


			// all agents observe
			for(size_t a=0; a<agents.size(); a++){
				if(a != na && agents[a].roleHistory[t] == 'e' && agents[a].market.contactWithObserver){ // exclude diff agent
					world.observe(agents[a].cLoc, agents[a].costmap);
					agents[a].market.updateMarket(agents[a].cLoc, agents[a].gLoc);
				}
			}

			tO.showCellsPlot();
			waitKey(1);
			tO.market.iterateTime();

		} // end 1 sim iter

		dRewards.push_back( gReward - getPercentObserved( world.costmap, tO.costmap) );
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

float getMean( vector<float> g){

	float sum = 0;
	for(size_t i=0; i<g.size(); i++){
		sum += g[i];
	}

	return sum / float( g.size() );

}

float getStdDev(float mean, vector<float> g){

	float sum = 0;
	for(size_t i=0; i<g.size(); i++){
		sum += pow( g[i] - mean, 2 );
	}
	sum = sum / float( g.size() );
	return sqrt( sum );

}
