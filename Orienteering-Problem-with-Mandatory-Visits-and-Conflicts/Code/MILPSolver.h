#pragma once
#include "Solver.h"

class MILPSolver : public Solver{
    IloEnv _env;
    IloCplex _cplex;
    IloModel _model;
    IloObjective _obj;

    IloArray< IloArray< IloNumVarArray > > _x;
    IloArray< IloNumVarArray > _y;
    IloArray< IloNumVarArray > _u;
    IloRangeArray _Constraints;

    void solvemethod(Solution* S);

public:

    MILPSolver(Instance* I, int OF);
    ~MILPSolver();

    double gap() { return _cplex.getMIPRelativeGap(); }
    double value() { return _cplex.getObjValue(); }

    Solution* recoversolution();
};

