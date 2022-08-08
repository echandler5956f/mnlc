// #pragma once
#ifndef hdbscan_H
#define hdbscan_H
#include <string>
#include <vector>
#include<iostream>
#include<fstream>
#include<sstream>
#include<set>
#include<map>
#include<cstdint>
#include "../Runner/hdbscanRunner.hpp"
#include "../Runner/hdbscanParameters.hpp"
#include "../Runner/hdbscanResult.hpp"
#include "../HdbscanStar/outlierScore.hpp"

using namespace std;

class Hdbscan

{

private:
	string fileName;

	hdbscanResult result;

public:
	vector<vector<double>> dataset;

	std::vector<int> labels_;

	std::vector<int> normalizedLabels_;

	std::vector<outlierScore> outlierScores_;

	std::vector<double> membershipProbabilities_;

	uint32_t noisyPoints_;

	uint32_t numClusters_;

	Hdbscan(vector<vector<double>> dataset)
	{

		this->dataset = dataset;
	}

	Hdbscan(string readFileName)
	{

		fileName = readFileName;
	}

	Hdbscan()
	{
	}

	string getFileName() {
	return this->fileName;
}

	int loadCsv(int numberOfValues, bool skipHeader) {
	string  attribute;

	string line = "";

	int currentAttributes;
	vector<vector<double> > dataset;

	string fileName = this->getFileName();
	ifstream file(fileName, ios::in);
	if (!file)
		return 0;
	if (skipHeader) {
		getline(file, line);

	}
	while (getline(file, line)) {      //Read through each line
		stringstream s(line);
		vector<double> row;
		currentAttributes = numberOfValues;
		while (getline(s, attribute, ',') && currentAttributes != 0) {
			row.push_back(stod(attribute));
			currentAttributes--;
		}
		dataset.push_back(row);

	}
	this->dataset = dataset;
	return 1;
}

void execute(int minPoints, int minClusterSize, string distanceMetric) {
	//Call The Runner Class here
	hdbscanRunner runner;
	hdbscanParameters parameters;
	uint32_t noisyPoints = 0;
	set<int> numClustersSet;
	map<int, int> clustersMap;
	vector<int> normalizedLabels;

	parameters.dataset = this->dataset;
	parameters.minPoints = minPoints;
	parameters.minClusterSize = minClusterSize;
	parameters.distanceFunction = distanceMetric;
    this->result = runner.run(parameters);
	this->labels_ = result.labels;
	this->outlierScores_ = result.outliersScores;
	for (uint32_t i = 0; i < result.labels.size(); i++) {
		if (result.labels[i] == 0) {
			noisyPoints++;
		}
		else {
			numClustersSet.insert(result.labels[i]);
		}
	}
	this->numClusters_ = numClustersSet.size();
	this->noisyPoints_ = noisyPoints;
	int iNdex = 1;
	for (auto it = numClustersSet.begin(); it != numClustersSet.end(); it++) {
		clustersMap[*it] = iNdex++;
	}
	for (int i = 0; i < labels_.size(); i++) {
		if (labels_[i] != 0)
			normalizedLabels.push_back(clustersMap[labels_[i]]);
		else if (labels_[i] == 0) {
			normalizedLabels.push_back(-1);
		}

	}
	this->normalizedLabels_ = normalizedLabels;
	this->membershipProbabilities_ = result.membershipProbabilities;
}

	void displayResult() {
	hdbscanResult result = this->result;
	uint32_t numClusters = 0;

	cout << "HDBSCAN clustering for " << this->dataset.size() << " objects." << endl;

	// for (uint32_t i = 0; i < this->normalizedLabels_.size(); i++) {
	// 	printf("%f\t%f\t%d\n", this->dataset[i][0], this->dataset[i][1], this->normalizedLabels_[i]);
	// 	// cout << this->dataset[i] << " ";
	// 	// cout << this->normalizedLabels_[i] << " ";
	// }

	// cout << endl << endl;

	cout << "The Clustering contains " << this->numClusters_ << " clusters with " << this->noisyPoints_ << " noise Points." << endl;

}
};

#endif