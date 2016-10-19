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

int findMapToRun(vector<string> &fName);
void loadMapNames(vector<string> &fName);
float getPercentObserved(Costmap &globalCostmap, Costmap &workingCostmap);

int main(){
	cerr << "Andy was here" << endl;

	destroyAllWindows();

	vector<int> treePath;
	srand( time(NULL) );
	bool videoFlag = false;

	int numAgents = 2;
	int numIterations = 1;

	int gSpace = 5;
	float obsThresh = 50;
	float comThresh = 50;
	int maxTime = 2000;

	vector<string> fName;
	//fName.push_back("mineMap");
	//fName.push_back("test6");
	//fName.push_back("slamMap");
	fName.push_back("tunnelTest");
	//loadMapNames(fName);

	int map_iter = 0;//findMapToRun(fName);
	cout << "fName.size(): " << fName.size() << " & mapIter: " << map_iter << endl;

	vector<string> exploreMethod;
	exploreMethod.push_back("greedy");
	//exploreMethod.push_back("select");


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

	for(size_t exploreMethod_iter = 0; exploreMethod_iter<exploreMethod.size(); exploreMethod_iter++){
		for(int iterations_iter = 0; iterations_iter<numIterations; iterations_iter++){

			Observer observer(sLoc[iterations_iter]);
			vector<Agent> agents;
			for(int i=0; i<numAgents; i++){
				agents.push_back(Agent(sLoc[iterations_iter*numAgents+i], i, world, obsThresh, comThresh, numAgents));
				cout << "Main::Agent[" << i << "]: created at : " << sLoc[iterations_iter*numAgents+i].x << " , " << sLoc[iterations_iter*numAgents+i].y << endl;
			}
			cout << "Main::All agents created" << endl;

			time_t start = clock();

			// video writers
			VideoWriter slamVideo;
			if(videoFlag){
				//Size frameSize( static_cast<int>(world.costmap.cells.cols, world.costmap.cells.rows) );
				//slamVideo.open("multiAgentInference.avi",CV_FOURCC('M','J','P','G'), 30, frameSize, true );
				cout << "Main::Videos started" << endl;
			}

			cout << "Main::Ready, press any key to begin." << endl;
			waitKey(1);
			cout << "Main::Here we go!" << endl;

			int timeSteps = -1;
			float percentObserved = 0;

			while(timeSteps < maxTime-1 && percentObserved < 0.99){
				cout << "Main::Starting while loop" << endl;
				timeSteps++;

				// all agents observe
				for(int i=0; i<numAgents; i++){
					world.observe(agents[i].cLoc, agents[i].costmap);
					agents[i].costmap.findFrontiers();
				}
				world.observe(observer.cLoc, observer.costmap);
				cout << "Main::made observations" << endl;

				// all agents communicate
				for(int i=0; i<numAgents-1; i++){
					for(int j=i; j<numAgents; j++){
						if(i!=j && world.communicationPossible(agents[i].cLoc, agents[j].cLoc)){
							agents[i].shareCostmap(agents[i].costmap, agents[j].costmap);
							agents[j].shareCostmap(agents[j].costmap, agents[i].costmap);
							agents[i].agentLocs[j] = agents[j].cLoc;
							agents[j].agentLocs[i] = agents[i].cLoc;

							//agents[i].market.locations[j] = agents[j].market.locations[j];
							//agents[j].market.locations[i] = agents[i].market.locations[i];

							//agents[i].market.costs[j] = agents[j].market.costs[j];
							//agents[j].market.costs[i] = agents[i].market.costs[i];


							//world.communicate(agents[i].cLoc, agents[i].costmap, agents[j].cLoc, agents[j].costmap);
						}
					}
				}
				cout << "Main::Out of communicate with other agents" << endl;

				// all agents communicate
				for(int i=0; i<numAgents; i++){
					agents[i].shareCostmap(agents[i].costmap, observer.costmap);
				}
				cout << "Main::Out of communicate with observer" << endl;

				observer.showCellsPlot(agents);
				waitKey(1);
				//observer.costmap.buildOccPlot();

				//waitKey(100);

				cout << "Main::Into plan for one timestep" << endl;
				// all agents plan for one timestep
				for(int i=0; i<numAgents; i++){
					agents[i].soloPlan(exploreMethod[exploreMethod_iter], maxTime - timeSteps); // includes updating internal cLoc
				}

				// all agents act for one timestep
				for(int i=0; i<numAgents; i++){
					agents[i].act();
				}

				// display observer view
				//observer.showCostmapPlot(agents);

				if(videoFlag){
					for(int i =0; i<5; i++){ // slower frmae rate through repeated frames
						slamVideo << observer.costmap.displayPlot;
					}
				}

				// print out progress
				percentObserved = getPercentObserved(world.costmap, agents[0].costmap);
				cout << "------timeSteps & percent observed: " << timeSteps << " & " << percentObserved << " " << map_iter << ": " << fName[map_iter] << endl;

			} // end timeStep in simulation

			cerr << "made it to the end of the simulation!" << endl;
		} // end iterations
	} // end explore methods
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

int findMapToRun(vector<string> &fName){

	/*
	ifstream file ( "/home/andy/git/coordination/Debug/InferenceSept10.csv", ifstream::in ); // declare file stream: http://www.cplusplus.com/reference/iostream/ifstream/
	if ( file.is_open() ){
		string value;
		vector<string> mapNames;
		string mapName;
		int c = 0;
		while ( file.good() ){
			c++;
			getline ( file, value, '\n' ); // read a string until next comma: http://www.cplusplus.com/reference/string/getline/

			int fnd = value.find_first_of(",");

			//if(!mapName.compare( value.substr(0, fnd)) == 0){
				//cout << value << endl; // display value removing the first and the last character from it
				mapName = value.substr(0, fnd);
				//cout << mapName << endl;
				mapNames.push_back( mapName );
			//}
		}
		file.close();

		vector<int> cnt;
		for(size_t i=0; i<fName.size(); i++){
			int fi = fName[i].find_last_of("/");
			string fn = fName[i].substr(fi+1);
			int cntr = 0;
			for(size_t j=0; j<mapNames.size(); j++){
				if( fn.compare( mapNames[j] ) == 0 ){
					cntr++;
				}
			}
			//cout << fn << " : " << cntr << endl;
			cnt.push_back(cntr);
		}

		double minV = INFINITY;
		vector<int> minI;

		for(size_t i=0; i<cnt.size(); i++){
			if(cnt[i] == minV){
				minV = cnt[i];
				minI.push_back(i);
			}
			if(cnt[i] < minV){
				minV = cnt[i];
				minI.clear();
				minI.push_back(i);
			}
		}


		int v = rand() % minI.size();

		return minI[v];
	}
	*/

	return rand() % fName.size();

}


void loadMapNames(vector<string> &fName){
	string dir = "/home/andy/git/fabmap2Test/InferenceLibrary/TestMaps/generated/";
	ifstream fin;
	string filepath;
	DIR *dp;
	struct dirent *dirp;
	struct filestat;

	dp = opendir( dir.c_str() ); // open provided directory
	if (dp == NULL){ // did it open
		cerr << "Error opening " << dir << endl;
		waitKey(0);
	}

	while ((dirp = readdir( dp ))){ // read all files in directory to get filenames
		// get file to open
		filepath = dir + dirp->d_name;
		string fileName = dirp->d_name;
		if(fileName.substr(fileName.find_last_of(".") + 1) != "jpg"){
		    continue;
		}
		fName.push_back( filepath.substr( 0, filepath.find_last_of(".") ) );
	}
}

float getPrecisionObserved(Costmap &workingCostmap, Costmap &globalCostmap){

	// precision = correctly observed / observed free

	int correctlyObsFree = 0;
	int obsFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				obsFree++;
				if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
					correctlyObsFree++;
				}
			}
		}
	}

	return float(correctlyObsFree)/float(obsFree);
}
