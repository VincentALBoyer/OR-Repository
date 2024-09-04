#pragma once
#include "Solver.h"

typedef SolverAlgo GreedyAlgo;


class Greedy :  public Solver
{
    Solution* _S;               //The best solution

    int _alpha;

    GreedyAlgo _A;

    void solvemethod();
    void solvemethod2();
    void initsolver(Solution* S = NULL);

public:

    Greedy(Instance* I, GreedyAlgo A=GreedyAlgo::Greedy1);

    ~Greedy();

    Solution* getSolution();
};

