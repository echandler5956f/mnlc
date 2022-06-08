// #pragma once
#ifndef ManhattanDistance_H
#define ManhattanDistance_H
#include"IDistanceCalculator.hpp"
#include<vector>
#include<cmath>
#include<cstdint>
/// <summary>
/// Computes the manhattan distance between two points, d = |x1-y1| + |x2-y2| + ... + |xn-yn|.
/// </summary>
class ManhattanDistance : IDistanceCalculator
{
public:
double computeDistance(std::vector<double> attributesOne, std::vector<double> attributesTwo) {
	double distance = 0;
	for (uint32_t i = 0; i < attributesOne.size() && i < attributesTwo.size(); i++) {
		distance += fabs(attributesOne[i] - attributesTwo[i]);
	}

	return distance;
}};

#endif