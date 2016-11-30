/*
 * Population.h
 *
 *  Created on: Nov 29, 2016
 *      Author: andy
 */

#ifndef POPULATION_H_
#define POPULATION_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <math.h>

using namespace std;

class Population {
public:
	Population(int popSize, int nConst);
	virtual ~Population();
	void mutate();
	void reset();

	vector<vector<float> > constants;
	int testIndex;
	vector<int> open;
	vector<float> rewards;
	int popSize;
	int nConst;
};

#endif /* POPULATION_H_ */
