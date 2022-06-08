// #pragma once
#ifndef hdbscanResult_H
#define hdbscanResult_H
#include<vector>
#include"../HdbscanStar/outlierScore.hpp"
using namespace std;
class hdbscanResult
{
public:
	vector <int> labels;
	vector <outlierScore> outliersScores;
	vector <double> membershipProbabilities;
	bool hasInfiniteStability;

	hdbscanResult() {
		;
	}

	hdbscanResult(vector<int> pLables, vector<outlierScore> pOutlierScores, vector<double> pmembershipProbabilities, bool pHsInfiniteStability) {
		labels = pLables;
		outliersScores = pOutlierScores;
		membershipProbabilities = pmembershipProbabilities;
		hasInfiniteStability = pHsInfiniteStability;
	}
};

#endif