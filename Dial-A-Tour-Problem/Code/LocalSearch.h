#pragma once
#include "Solver.h"
class LocalSearch : public Solver
{
    Solver* _RepairHeur;

    Solution* _S;

    void solvemethod();
    void initsolver(Solution* S = NULL);

    Solution* getBestNeiborhood(Solution *S);

    void improveOperationsPositioning(Solution* S);

    void perturbation(Solution* S, double p = 0.5);


public:

    LocalSearch(Solver* RepairHeuristic);

    ~LocalSearch();

    Solution* getSolution();
};

