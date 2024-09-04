#include "MILPSolver.h"


void MILPSolver::solvemethod()
{
    _cplex.solve(); 
	cout << "Status: " << _cplex.getStatus() << endl;
    if (_cplex.getStatus() != IloAlgorithm::InfeasibleOrUnbounded && _cplex.getStatus() != IloAlgorithm::Unknown)
        _gap = _cplex.getMIPRelativeGap();

	_I->setLB(_cplex.getBestObjValue());

}

void MILPSolver::initsolver(Solution* S)
{
	//Model Initialization
	_nvehicles = fmin(_nvehicles, _I->ngroups());
	if(S!=NULL) _nvehicles = S->fitness();
	if (_prevnvehicles != _nvehicles) {
		_env.end();
		initmodel();
	}
	_prevnvehicles = _nvehicles;

	//MILPStart
    if (S != NULL) {

        IloNumVarArray Var(_env);
        IloNumArray Val(_env);

        int v = 0;
        for (int i = 0; i < S->nop(); ++i) {
            operation* o = S->getop(i);
            if (S->prevvehicleop(o) == NULL) {
                assert(v < _nvehicles);
                Var.add(_x[v][initialdepotid()][S->getid(o)]);
                Val.add(1);
                assert(i == S->getid(o));
                while (S->nextvehicleop(o) != NULL) {
                    int opid = S->getid(o);
                    int nextopid = S->getid(S->nextvehicleop(o));
                    int st = S->st(o);
                    Var.add(_x[v][opid][nextopid]);
                    Val.add(1);

                    o = S->nextvehicleop(o);
                }
                Var.add(_x[v][S->getid(o)][finaldepotid()]);
                Val.add(1);
                v++;

            }
        }

        _cplex.addMIPStart(Var, Val, IloCplex::MIPStartRepair);
        
        Var.end();
        Val.end();
    }

    _cplex.setParam(IloCplex::TiLim, _TimLim);
    _cplex.setParam(IloCplex::EpGap, 0.01);
    _cplex.setParam(IloCplex::Threads, 1);
    
}

void MILPSolver::initmodel()
{
    try {
		_env = IloEnv();
		_M = _I->planhor();
		_model = IloModel(_env);
		_cplex = IloCplex(_model);
		_obj = IloAdd(_model, IloMinimize(_env, 0));

		_Constraints = IloRangeArray(_env);

		//Variables x
		_x = IloArray < IloArray<IloNumVarArray > >(_env, _nvehicles);
		for (int v = 0; v < _nvehicles; ++v) {
			_x[v] = IloArray<IloNumVarArray >(_env, nnodes());
			for (int i = 0; i < nnodes(); ++i) {
				_x[v][i] = IloNumVarArray(_env);
				for (int j = 0; j < nnodes(); ++j) {
					string label = "x_" + to_string(v) + "_" + to_string(i) + "_" + to_string(j);
					_x[v][i].add(IloBoolVar(_env, label.c_str()));
				}
			}
		}

		//Variables q
		_q = IloNumVarArray(_env);
		for (int i = 0; i < nnodes(); ++i) {
		    string label = "q_" + to_string(i);
		    _q.add(IloNumVar(_env, 0, _I->vcapacity(), ILOFLOAT, label.c_str()));
		}
		_q[initialdepotid()].setUB(0);

		//Variables s
		_s = IloNumVarArray(_env);
		for (int i = 0; i < nnodes(); ++i) {
		    string label="s_"+to_string(i);
		    _s.add(IloNumVar(_env, 0, _I->planhor(), ILOFLOAT, label.c_str()));
		}

		if (_MILPModel == MILPModel::MILP1) genMILP1();
		else if(_MILPModel == MILPModel::MILP2) genMILP2();
		else if(_MILPModel == MILPModel::MILP3) genMILP3();
		else {
			cout << "Error: MILP Model unknown...exiting" << endl;
			exit(1001);
		}

		_model.add(_Constraints);

		_model.add(_obj>=_I->LB());

	}
	catch (IloException& ex) {
		cerr << "Error: " << ex << endl;
	}
}


void MILPSolver::genMILP3()
{
	//Objective
	IloExpr obj(_env);
	for (int v = 0; v < _nvehicles; ++v) {
		int o = initialdepotid();
		int d = finaldepotid();
		obj += _x[v][o][d];
	}
	_obj.setExpr(_nvehicles - obj);
	obj.end();

	addInitialDepotDepartureConstraints();
	addFlowConservationConstraints();
	addCutOnFlowConstraints();
	addMandatoryVisitsConstraints();
	addTimeConsistencyConstraints();
	addPrecedenceConstraints();
	addQoSConstraints();
	addPickupAndDeliveryConstraints();
	addVehicleCapacityConstraints();
	addMTZSubTourEliminationConstraints();
}

void MILPSolver::addCutOnFlowConstraints()
{
	for (int v = 0; v < _nvehicles; ++v) {
		for (int i = 0; i < nnodes(); ++i) if (!isdepot(i)) {
			string labelin = "FCIN_" + to_string(v) + "_" + to_string(i);
			IloExpr exprin(_env);
			string labelout = "FCOUT_" + to_string(v) + "_" + to_string(i);
			IloExpr exprout(_env);
			for (int j = 0; j < nnodes(); ++j) if (i != j) {
				if (j != finaldepotid()) exprin += _x[v][j][i];
				if (j != initialdepotid()) exprout -= _x[v][i][j];
			}
			_Constraints.add(IloRange(_env, -IloInfinity, exprin+_x[v][initialdepotid()][finaldepotid()], 1, labelin.c_str()));
			exprin.end();
			exprout.end();
		}
	}
}

void MILPSolver::addCutOnInitialDeparture()
{
	for (int v = 0; v < _nvehicles; ++v) {
		int o = initialdepotid();
		int d = finaldepotid();
		
		IloExpr expr(_env);
		for (int i = 0; i < nnodes(); ++i) if (!isdepot(i))  expr += _x[v][o][i];

		for (int i = 0; i < nnodes(); ++i) if (i != finaldepotid()) {
			for (int j = 0; j < nnodes(); ++j) if (i != j && !isdepot(j)) {
				string label = "CutInit_" + to_string(v)+"_"+to_string(i)+"_"+to_string(j);
				_Constraints.add(IloRange(_env, -IloInfinity, _x[v][i][j] - expr, 0,label.c_str()));
			}
		}

		expr.end();
	}
}

MILPSolver::MILPSolver(Instance* I, unsigned int nvehicles) : Solver(I, Parameter::tostring(MILPModel::MILP3))
{
	_MILPModel = MILPModel::MILP1;

	_prevnvehicles = INT_MAX;
	_nvehicles = nvehicles;

}

MILPSolver::MILPSolver(Instance* I, MILPModel M, unsigned int nvehicles) : Solver(I, Parameter::tostring(M) +"Solver")
{
	_MILPModel = M;

	_prevnvehicles = INT_MAX;
	_nvehicles = nvehicles;

}

MILPSolver::~MILPSolver()
{
    _env.end();
}

Solution* MILPSolver::getSolution()
{
    if (_cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded || _cplex.getStatus() == IloAlgorithm::Unknown) {
        return nullptr;
    }

    Solution* S = new Solution(_I);

    for (int i = 0; i < nnodes(); ++i) if (!isdepot(i)) {
        for (int j = 0; j < nnodes(); ++j) if (!isdepot(j) && i!=j) {
            for (int v = 0; v < _nvehicles; ++v) {
                if (_cplex.getValue(_x[v][i][j]) >= 0.9999) {
                    operation* o = S->getop(i);
                    operation* d = S->getop(j);
                    S->createvehiclelink(o, d);
                }
            }
        }
    }

    for (int i = 0; i < nnodes(); ++i) if (!isdepot(i)) {
        operation* o = S->getop(i);
        int st = (_cplex.getValue(_s[i]));
        int q = (_cplex.getValue(_q[i]));
        S->setst(o, st);
        S->setvcap(o, q);
    }

    assert(S->update());

    assert(S->isFeasible(true));

    return S;
}

inline int MILPSolver::distance(int o, int d)
{
	if (isdepot(o) || isdepot(d)) return 0;
	else return _I->travellingtime(o, d);
}

void MILPSolver::addInitialDepotDepartureConstraints()
{
	for (int v = 0; v < _nvehicles; ++v) {
		int o = initialdepotid();
		int d = finaldepotid();
		string label = "init_" + to_string(v);
		IloExpr expr(_env);
		for (int i = 0; i < nnodes(); ++i) if (!isdepot(i))  expr += _x[v][o][i];

		_Constraints.add(IloRange(_env, 1, expr + _x[v][o][d], 1.0, label.c_str()));

		expr.end();
	}
}

void MILPSolver::addFlowConservationConstraints()
{
	for (int v = 0; v < _nvehicles; ++v) {
		for (int i = 0; i < nnodes(); ++i) if (!isdepot(i)) {
			string label = "flc_" + to_string(v) + "_" + to_string(i);
			IloExpr expr(_env);
			for (int j = 0; j < nnodes(); ++j) if (i != j) {
				if (j != finaldepotid()) expr += _x[v][j][i];
				if (j != initialdepotid()) expr -= _x[v][i][j];
			}
			_Constraints.add(IloRange(_env, 0, expr, 0, label.c_str()));
			expr.end();
		}
	}
}

void MILPSolver::addMandatoryVisitsConstraints()
{
	//Note: it needs the flow conservation
	for (int i = 0; i < nnodes(); ++i) if (!isdepot(i)) {
		string labelin = "infl_" + to_string(i);
		string labelout = "outfl_" + to_string(i);
		IloExpr exprin(_env);
		IloExpr exprout(_env);
		for (int j = 0; j < nnodes(); ++j) if (i != j) {
			for (int v = 0; v < _nvehicles; ++v) {
				if (j != finaldepotid()) exprin += _x[v][j][i];
				if (j != initialdepotid()) exprout += _x[v][i][j];
			}
		}
		_Constraints.add(IloRange(_env, 1, exprin, 1, labelin.c_str()));
		_Constraints.add(IloRange(_env, 1, exprout, 1, labelout.c_str()));
		exprin.end();
		exprout.end();
	}
}

void MILPSolver::addTimeConsistencyConstraints()
{
	for (int i = 0; i < nnodes(); ++i) if (!isdepot(i)) {
		for (int j = 0; j < nnodes(); ++j) if (i != j && !isdepot(j)) {
			string label = "t_" + to_string(i) + "_" + to_string(j);
			IloExpr expr(_env);
			for (int v = 0; v < _nvehicles; ++v) expr += _x[v][i][j];
			int d = distance(i, j);
			_Constraints.add(IloRange(_env, -IloInfinity, _s[i] - _s[j] - (_M + d) * (1 - expr), -d, label.c_str()));
			expr.end();
		}
	}
}

void MILPSolver::addPrecedenceConstraints()
{
	for (int i = 0; i < nnodes(); ++i) if (!isdepot(i) && !_I->islastnode(i)) {
		int j = _I->nextnodeid(i);

		string label = "t_g" + to_string(_I->groupid(i)) + "_" + to_string(i) + "_" + to_string(j);
		int d = distance(i, j);
		assert(d >= 0);
		_Constraints.add(IloRange(_env, -IloInfinity, _s[i] - _s[j], -d, label.c_str()));
	}
}

void MILPSolver::addQoSConstraints()
{
	for (int g = 0; g < _I->ngroups(); ++g) {
		int i = _I->firstgroupnode(g);
		int j = i;
		while (!_I->islastnode(j)) j = _I->nextnodeid(j);
		string label = "QoS_" + to_string(_I->groupid(i));
		_Constraints.add(IloRange(_env, -IloInfinity, _s[j] - _s[i], _I->maxtourlength(g), label.c_str()));
	}
}

void MILPSolver::addPickupAndDeliveryConstraints()
{
	for (int i = 0; i < nnodes(); ++i) if (!isdepot(i) && _I->ispickupnode(i)) {
		assert(!_I->islastnode(i));
		int j = _I->nextnodeid(i);
		assert(_I->isdeliverynode(j));

		for (int v = 0; v < _nvehicles; ++v) {
			string label = "sv_" + to_string(v) + "_" + to_string(i) + "_" + to_string(j);
			IloExpr expr(_env);
			for (int k = 0; k < nnodes(); ++k) {
				if (k != finaldepotid() && i != k) expr += _x[v][k][i];
				if (k != finaldepotid() && j != k) expr -= _x[v][k][j];
			}
			_Constraints.add(IloRange(_env, 0, expr, 0, label.c_str()));
			expr.end();
		}
	}
}

void MILPSolver::addVehicleCapacityConstraints()
{
	//Load propagation
	for (int i = 0; i < nnodes(); ++i) if (i != finaldepotid()) {
		for (int j = 0; j < nnodes(); ++j) if (j != i && !isdepot(j)) {
			string label = "q_" + to_string(i) + "_" + to_string(j);
			IloExpr expr(_env);
			for (int v = 0; v < _nvehicles; ++v) expr += _x[v][i][j];

			double load = 0;
			if (j != finaldepotid()) {
				int groupid = _I->groupid(j);
				if (_I->ispickupnode(j)) load = _I->groupsize(groupid);
				else if (_I->isdeliverynode(j)) load = -_I->groupsize(groupid);
			}
			int M = _I->vcapacity() + fabs(load);

			_Constraints.add(IloRange(_env, -IloInfinity, _q[i] - _q[j] - M * (1 - expr), -load, label.c_str()));
			expr.end();
		}
	}
}

void MILPSolver::addMTZSubTourEliminationConstraints()
{
	//Note: Constraints are added only when distan (i,j) is 0
	IloNumVarArray u(_env);
	for (int i = 0; i < nnodes(); ++i) {
		string label = "u_" + to_string(i);
		u.add(IloNumVar(_env, 1, nnodes() - 1, ILOINT, label.c_str()));
	}
	u[initialdepotid()].setBounds(0, 0);
	u[finaldepotid()].setBounds(nnodes() - 1, nnodes() - 1);

	for (int i = 0; i < nnodes(); ++i) if (i != finaldepotid() && i != initialdepotid()) {
		for (int j = 0; j < nnodes(); ++j) if (j != i && j != initialdepotid() && distance(i, j) <= 0.001) {
			IloExpr expr(_env);
			for (int v = 0; v < _nvehicles; ++v) expr += _x[v][i][j];

			_Constraints.add(IloRange(_env, -IloInfinity, u[i] - u[j] - (nnodes() - 1) * (1 - expr), -1));
			expr.end();

		}
	}
}
