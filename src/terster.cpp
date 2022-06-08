#include<iostream>
#include<cstdio>
#include "hdbscan/Hdbscan/hdbscan.hpp"
using namespace std;


int main() {
	Hdbscan hdbscan("FourProminentClusterDataset.csv");
	hdbscan.loadCsv(2, false);
	vector<vector <double>> dataset = hdbscan.dataset;
	hdbscan.execute(5, 5, "Euclidean");
	hdbscan.displayResult();
	cout << "You can access other fields like cluster labels, membership probabilities and outlier scores."<<endl;
	/*Use it like this
	hdbscan.labels_;
	hdbscan.membershipProbabilities_;
	hdbscan.outlierScores_;
	*/

	return 0;
}