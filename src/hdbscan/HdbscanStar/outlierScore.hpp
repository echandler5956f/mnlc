// #pragma once
#ifndef outlierScore_H
#define outlierScore_H
#include <tuple> 

	/// <summary>
	/// Simple storage class that keeps the outlier score, core distance, and id (index) for a single point.
	/// OutlierScores are sorted in ascending order by outlier score, with core distances used to break
	/// outlier score ties, and ids used to break core distance ties.
	/// </summary>
class outlierScore
{
private:
	double coreDistance;
public:
	double score;
	int id;
	/// <summary>
	/// Creates a new OutlierScore for a given point.
	/// </summary>
	/// <param name="score">The outlier score of the point</param>
	/// <param name="coreDistance">The point's core distance</param>
	/// <param name="id">The id (index) of the point</param>
outlierScore(double score, double coreDistance, int id) {
	this->score = score;
	this->coreDistance = coreDistance;
	this->id = id;
}	
outlierScore() {
}
	/// <summary>
	/// Method Overridden to compare two objects.
	/// </summary>
bool operator<(const outlierScore& other) const {
	/*
	if (score < other.score)
		return score < other.score;
	else if (coreDistance < other.coreDistance)
		return coreDistance < other.coreDistance;
	else if (id < other.id)
		return id < other.id;
	else
		return false;*/
	return std::tie(score, coreDistance, id) < std::tie(other.score, other.coreDistance, other.id);
}

};

#endif