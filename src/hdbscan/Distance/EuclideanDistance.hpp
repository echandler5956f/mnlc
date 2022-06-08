// #pragma once
#ifndef EuclideanDistance_H
#define EuclideanDistance_H
#include"IDistanceCalculator.hpp"
#include<vector>
#include<cmath>
#include<cstdint>
/// <summary>
/// Computes the euclidean distance between two points, d = sqrt((x1-y1)^2 + (x2-y2)^2 + ... + (xn-yn)^2).
/// </summary>
class EuclideanDistance : IDistanceCalculator
{
public:
double computeDistance(std::vector<double> attributesOne, std::vector<double> attributesTwo) {
	double distance = 0;
	for (uint32_t i = 0; i < attributesOne.size() && i < attributesTwo.size(); i++) {
		distance += ((attributesOne[i] - attributesTwo[i]) * (attributesOne[i] - attributesTwo[i]));
	}

	return sqrt(distance);
}
};

#endif