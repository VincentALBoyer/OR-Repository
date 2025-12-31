#pragma once
#include "Instance.h"
class Solution : public vector<int>{
	Instance* _I;

public:
	Solution(Instance* I);
	~Solution();
    
    double fitness();

	bool isFeasible();
    
    void print();
};

