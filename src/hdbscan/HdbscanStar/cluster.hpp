// #pragma once
#ifndef cluster_H
#define cluster_H
#include <set>
#include <vector>
#include <limits>
#include <stdexcept>
#include <iostream>


class cluster
{
private:
	int _id;
	double _birthLevel;
	double _deathLevel;
	int _numPoints;
	double _propagatedStability;
	int _numConstraintsSatisfied;
	int _propagatedNumConstraintsSatisfied;
	std::set<int> _virtualChildCluster;

public:
	int counter;
	std::vector<cluster*> PropagatedDescendants;
	double PropagatedLowestChildDeathLevel;
	cluster* Parent;
	double Stability;
	bool HasChildren;
	int Label;
	int HierarchyPosition;   //First level where points with this cluster's label appear


cluster()
{
	_id = ++counter;
}

cluster(int label, cluster* parent, double birthLevel, int numPoints) //:Label(label), Parent(parent), _birthLevel(birthLevel), _numPoints(numPoints)
{
	_id = ++counter;
	_deathLevel = 0;

	_propagatedStability = 0;
	_numConstraintsSatisfied = 0;
	_propagatedNumConstraintsSatisfied = 0;

	Parent = parent;
	Label = label;
	_birthLevel = birthLevel;
	_numPoints = numPoints;
	HierarchyPosition = 0;
	Stability = 0;
	PropagatedLowestChildDeathLevel = std::numeric_limits<double>::max();

	if (Parent != NULL)
		Parent->HasChildren = true;
	HasChildren = false;
	PropagatedDescendants.resize(0);
}

bool operator==(const cluster& other) const {
	return (this->_id == other._id);
}

void detachPoints(int numPoints, double level)
{
	_numPoints -= numPoints;
	Stability += (numPoints * (1 / level - 1 / _birthLevel));

	if (_numPoints == 0)
		_deathLevel = level;
	else if (_numPoints < 0)
		throw std::invalid_argument("Cluster cannot have less than 0 points.");
}

void propagate()
{
	if (Parent != NULL)
	{
		if (PropagatedLowestChildDeathLevel == std::numeric_limits<double>::max())
			PropagatedLowestChildDeathLevel = _deathLevel;
		if (PropagatedLowestChildDeathLevel < Parent->PropagatedLowestChildDeathLevel)
			Parent->PropagatedLowestChildDeathLevel = PropagatedLowestChildDeathLevel;
		if (!HasChildren)
		{
			Parent->_propagatedNumConstraintsSatisfied += _numConstraintsSatisfied;
			Parent->_propagatedStability += Stability;
			Parent->PropagatedDescendants.push_back(this);
		}
		else if (_numConstraintsSatisfied > _propagatedNumConstraintsSatisfied)
		{
			Parent->_propagatedNumConstraintsSatisfied += _numConstraintsSatisfied;
			Parent->_propagatedStability += Stability;
			Parent->PropagatedDescendants.push_back(this);
		}
		else if (_numConstraintsSatisfied < _propagatedNumConstraintsSatisfied)
		{
			Parent->_propagatedNumConstraintsSatisfied += _propagatedNumConstraintsSatisfied;
			Parent->_propagatedStability += _propagatedStability;
			Parent->PropagatedDescendants.insert(Parent->PropagatedDescendants.end(), PropagatedDescendants.begin(), PropagatedDescendants.end());
		}
		else if (_numConstraintsSatisfied == _propagatedNumConstraintsSatisfied)
		{
			//Chose the parent over descendants if there is a tie in stability:
			if (Stability >= _propagatedStability)
			{
				Parent->_propagatedNumConstraintsSatisfied += _numConstraintsSatisfied;
				Parent->_propagatedStability += Stability;
				Parent->PropagatedDescendants.push_back(this);
			}
			else
			{
				Parent->_propagatedNumConstraintsSatisfied += _propagatedNumConstraintsSatisfied;
				Parent->_propagatedStability += _propagatedStability;
				Parent->PropagatedDescendants.insert(Parent->PropagatedDescendants.end(), PropagatedDescendants.begin(), PropagatedDescendants.end());
			}
		}
	}
}

void addPointsToVirtualChildCluster(std::set<int> points)
{
	for (std::set<int>::iterator it = points.begin(); it != points.end(); ++it) {
		_virtualChildCluster.insert(*it);
	}
}
	
bool virtualChildClusterConstraintsPoint(int point)
{
	return (_virtualChildCluster.find(point) != _virtualChildCluster.end());
}

	void addVirtualChildConstraintsSatisfied(int numConstraints)
{
	_propagatedNumConstraintsSatisfied += numConstraints;
}
	

void addConstraintsSatisfied(int numConstraints)
{
	_numConstraintsSatisfied += numConstraints;
}


	void releaseVirtualChildCluster()
{
	_virtualChildCluster.clear();
}

	int getClusterId() {
	return this->_id;
}


};
#endif