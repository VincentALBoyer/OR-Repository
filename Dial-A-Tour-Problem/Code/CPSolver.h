#pragma once
#include "Solver.h"

typedef SolverAlgo CPModel;

class CPSolver :  public Solver
{
    int _prevnvehicles;								//Use to detect change in # of vehicles
    int _nvehicles;

    CPModel _CPModel;

    IloEnv _env;                                     //Environnement Cplex
    IloCP _cp;
    IloModel _model;
    IloObjective _obj;                              //objective

    IloTransitionDistance _M;                       //Distance matrix
    IloIntArray _type;                              //Node type for the distance matrix
    IloIntervalSequenceVarArray _R;                 //Routes of vehicles   
    IloIntervalVarArray _spanv;                     //span over the operations of a vehicle
    IloIntervalVarArray _operations;                //All the loading and unloading operations
    IloIntervalVarArray2 _opalt;                     //The operations alternatives

    IloIntVarArray _z;                              //The vehicle assignment for Model 2


    int _nop;

    int gettype(int opid);

    void solvemethod();
    void initsolver(Solution* S = NULL);
    void initmodel();


public:

	CPSolver(Instance* I, unsigned int nvehicles=INT_MAX);

    CPSolver(Instance* I,CPModel M, unsigned int nvehicles = INT_MAX);


	~CPSolver();

    void genModel1();

    void genModel2();

    int nop() { return _nop; }

    Solution* getSolution();
};

