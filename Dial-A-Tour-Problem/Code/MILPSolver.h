#pragma once
#include "Solver.h"


typedef SolverAlgo MILPModel;

class MILPSolver : public Solver
{
	int _prevnvehicles;								//Use to detect change in # of vehicles
	int _nvehicles;	
    double _M;

    IloEnv _env;                                     //Environnement Cplex
    IloCplex _cplex;
    IloModel _model;        
    IloObjective _obj;                               //objective

    IloArray < IloArray<IloNumVarArray >> _x;        //sequence variables i/j

    IloNumVarArray _q;				                 //operation to machine assignment m/i

    IloNumVarArray _s;                               //starting time of operation i


    int _nconstraints;                               //total number of constrants
    IloRangeArray _Constraints;                      //All the constraints

	MILPModel _MILPModel;


	void solvemethod();
	void initsolver(Solution* S = NULL);
	void initmodel();

	void genMILP3();

	void addInitialDepotDepartureConstraints();
	void addFlowConservationConstraints();
	void addMandatoryVisitsConstraints();
	void addTimeConsistencyConstraints();
	void addPrecedenceConstraints();
	void addQoSConstraints();
	void addPickupAndDeliveryConstraints();
	void addVehicleCapacityConstraints();
	void addMTZSubTourEliminationConstraints();

	void addCutOnFlowConstraints();
	void addCutOnInitialDeparture();
	

public:

	MILPSolver(Instance* I, unsigned int nvehicles = INT_MAX);

	MILPSolver(Instance* I, MILPModel M , unsigned int nvehicles = INT_MAX);

	~MILPSolver();

	Solution* getSolution();

	int nnodes() { return _I->nnodes() + 2; }

	bool isdepot(int nodeid) { return nodeid >= _I->nnodes(); }
	int initialdepotid() { return _I->nnodes(); }
	int finaldepotid() { return _I->nnodes()+1; }

	int distance(int o, int d);
	double getXValue(int v, int i, int j) { return _cplex.getValue(_x[v][i][j]); }
};

