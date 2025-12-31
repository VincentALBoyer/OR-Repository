#pragma once
#include "Instance.h"

class Solution{
	Instance* _I;
	list<int>* _route;

public:
	Solution(Instance* I);
	~Solution();

	Instance* getInstance() { return _I; }
	bool isFeasible(bool saveinfile=false);
	void printSolution();
	void saveSolution(double, int);
	void addNode(int, int);
	void addNode_pos(int, int, int);
	double totalScoreK(int k);
	double minScore();
	double maxDev();
	int get_ScoreAcum(int k);
	double get_Tacum(int k);
	list<int> getRoute(int);
    //double fitness();
};

