#pragma once
#include "Solver.h"

class CPSolver : public Solver{
    IloEnv _env;
    IloCP _cp;
    IloModel _model;
    IloObjective _obj;

    IloTransitionDistance _M;           //Distance matrix
    IloIntervalSequenceVarArray _R;     //Sequence of a route
    IloIntervalVarArray _visits;        //The visit to a customer
    IloIntervalVarArray2 _visitalts;    //Alternative visits
    IloIntervalVarArray _routespans;    //Span of a route

    void solvemethod(Solution* S);

public:
    CPSolver(Instance* I, int of, int version);
    ~CPSolver();
    void initSolver(Solution* );

    double value() { return _cp.getObjValue(); }
    double gap() { return _cp.getObjGap(); } // https://www.ibm.com/docs/en/icos/20.1.0?topic=cp-objgap-property

    Solution* recoversolution();
};

