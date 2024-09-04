#include "CPSolver.h"

int CPSolver::gettype(int opid)
{
    int i = _I->spotid(opid);
    for (int j = 0; j < i; ++j) {
        int tr = _I->travellingtime(_I->getspot(i), _I->getspot(j));
        if (tr <= 0) return j;
    }
    return i;
}

void CPSolver::solvemethod()
{
    try {
        _cp.solve();
        if (_cp.getStatus() != IloAlgorithm::InfeasibleOrUnbounded && _cp.getStatus() != IloAlgorithm::Infeasible)
            _gap = _cp.getObjGap();

        _I->setLB(_cp.getObjBound());
    }
    catch (IloException& e) {
        cout << e << endl;
        exit(1001);
    }
}

void CPSolver::initsolver(Solution* S)
{
    //Model Initialization
    _nvehicles = fmin(_nvehicles, _I->ngroups());
    if (S != NULL) _nvehicles = S->fitness();
    if (_prevnvehicles != _nvehicles) {
        _env.end();
        initmodel();
    }
    _prevnvehicles = _nvehicles;

    //Starting Point
    if (S != NULL) {

        IloSolution IloS(_env);

        int v = 0;
        for (int i = 0; i < nop(); ++i) {
            operation* o = S->getop(i);
            if (S->prevvehicleop(o) == NULL) {
                assert(v < _nvehicles);
                while (o != NULL) {
                    int opid = S->getid(o);
                    int st = S->st(o);
                    if (_CPModel == CPModel::CP1) {
                        
                        IloS.add(_opalt[opid][v]);
                        IloS.setPresent(_opalt[opid][v]);
                        IloS.setStart(_opalt[opid][v], st);
                        IloS.setEnd(_opalt[opid][v], st);
                        IloS.setStart(_operations[opid], st);
                        IloS.setEnd(_operations[opid], st);
                        
                    }
                    else if (_CPModel == CPModel::CP2 || _CPModel == CPModel::CP3) {
                        if (_I->ispickupnode(opid)) {
                            operation* onext = S->nextgroupop(o);
                            assert(onext != NULL && S->isdeliveryop(onext));
                            int et = S->st(onext);
                            IloS.setStart(_operations[opid], st);
                            IloS.setEnd(_operations[opid], et);

                            IloS.add(_z[opid]);
                            IloS.setValue(_z[opid], v);
                        }
                    }
                    o = S->nextvehicleop(o);
                }
                v++;
                
            }
        }
      
        _cp.setStartingPoint(IloS);
    }

    _cp.setParameter(IloCP::TimeLimit, _TimLim);
    _cp.setParameter(IloCP::FailLimit, IloIntMax);
    _cp.setParameter(IloCP::Workers, 1);
    _cp.setParameter(IloCP::SearchType, IloCP::Restart);
}

void CPSolver::initmodel()
{
    _env = IloEnv();
    _model = IloModel(_env);
    _cp = IloCP(_model);

    

    if (_CPModel == CPModel::CP1) genModel1();
    else if (_CPModel == CPModel::CP2) genModel2();
    else if (_CPModel == CPModel::CP3) genModel3();
    else {
        cout << "Error: Selected Model is Unknown for CP Solver" << endl;
        exit(222);
    }

    _model.add(_obj>=_I->LB());
    
}


CPSolver::CPSolver(Instance* I, unsigned int nvehicles):Solver(I, Parameter::tostring(CPModel::CP2))
{
    _CPModel = CPModel::CP2;
    _nop = _I->nnodes();

    _prevnvehicles = INT_MAX;
    _nvehicles = nvehicles;

}

CPSolver::CPSolver(Instance* I, CPModel M, unsigned int nvehicles) :Solver(I, Parameter::tostring(M) + "Solver")
{
    _CPModel = M;
    _nop = _I->nnodes();

    _prevnvehicles = INT_MAX;
    _nvehicles = nvehicles;
}

CPSolver::~CPSolver()
{
    _env.end();
}

void CPSolver::genModel1()
{
    _nop = _I->nnodes();

    //The operations
    _operations = IloIntervalVarArray(_env);
    _opalt = IloIntervalVarArray2(_env, nop());
    _type = IloIntArray(_env, nop());

    for (int i = 0; i < nop(); ++i) {
        _type[i] = gettype(i);//_I->spotid(i);
        string label = "op_" + to_string(i);
        IloIntervalVar op(_env, label.c_str());
        op.setLengthMax(0);
        op.setEndMax(_I->planhor());
        op.setPresent();
        _operations.add(op);

        //The alternatives
        _opalt[i] = IloIntervalVarArray(_env);
        for (int v = 0; v < _nvehicles; ++v) {
            string labelalt = "opa_" + to_string(i) + "_" + to_string(v);
            IloIntervalVar opalt(_env, labelalt.c_str());
            opalt.setLengthMax(0);
            opalt.setEndMax(_I->planhor());
            opalt.setOptional();
            _opalt[i].add(opalt);
        }
    }

    //Distance Matrix
    _M = IloTransitionDistance(_env, _I->nspots());
    for (int i = 0; i < _I->nspots(); ++i) {
        for (int j = 0; j < _I->nspots(); ++j) {
            int dist = _I->travellingtime(_I->getspot(i), _I->getspot(j));
            _M.setValue(i, j, dist);
        }

    }

    //Vehicle Route Sequence and span
    _R = IloIntervalSequenceVarArray(_env);
    _spanv = IloIntervalVarArray(_env);
    for (int v = 0; v < _nvehicles; ++v) {
        string label = "r_" + to_string(v);
        IloIntervalVarArray a(_env);
        for (int o = 0; o < nop(); ++o)  a.add(_opalt[o][v]);
        _R.add(IloIntervalSequenceVar(_env, a, _type, label.c_str()));

        string name = "sv_" + to_string(v);
        IloIntervalVar s(_env, name.c_str());
        s.setOptional();
        s.setEndMax(_I->planhor());
        _spanv.add(s);

        _model.add(IloSpan(_env, s, a));
    }

    //Objective
    IloNumExpr objexpr(_env);
    for (int v = 0; v < _nvehicles; ++v)
        objexpr += IloPresenceOf(_env, _spanv[v]);
    _obj = IloMinimize(_env, objexpr);
    _model.add(_obj);

    //Operations alternative
    for (int o = 0; o < nop(); ++o) {
        string label = "Alt_" + to_string(o);
        _model.add(IloAlternative(_env, _operations[o], _opalt[o], label.c_str()));
    }

    //NoOverLap for vehicles
    for (int v = 0; v < _nvehicles; ++v) {
        string label = "NOL_" + to_string(v);
        _model.add(IloNoOverlap(_env, _R[v], _M, true, label.c_str()));
    }

    //Group sequence and stay length
    for (int g = 0; g < _I->ngroups(); ++g) {
        int o = _I->firstgroupnode(g);
        while (!_I->islastnode(o)) {
            int nexto = _I->nextnodeid(o);
            int length = _I->travellingtime(o, nexto);
            _model.add(IloEndBeforeStart(_env, _operations[o], _operations[nexto], length));
            o = nexto;
        }
        _model.add(IloStartBeforeEnd(_env, _operations[o], _operations[_I->firstgroupnode(g)], -_I->maxtourlength(g)));
    }

    //load and unload must be done by the same vehicle
    IloCumulFunctionExprArray C(_env, _nvehicles);
    for (int v = 0; v < _nvehicles; ++v) C[v] = IloCumulFunctionExpr(_env);
    for (int g = 0; g < _I->ngroups(); ++g) {
        int o = _I->firstgroupnode(g);
        while (!_I->islastnode(o)) {
            int nexto = _I->nextnodeid(o);
            if (_I->ispickupnode(o)) {
                assert(_I->isdeliverynode(nexto));
                for (int v = 0; v < _nvehicles; ++v) {
                    _model.add(IloIfThen(_env, IloPresenceOf(_env, _opalt[o][v]), IloPresenceOf(_env, _opalt[nexto][v])));
                    C[v] += IloStepAtStart(_opalt[o][v], _I->groupsize(g));
                    C[v] -= IloStepAtEnd(_opalt[nexto][v], _I->groupsize(g));
                }
            }
            o = nexto;
        }
    }
    //Vehicle Capacity
    for (int v = 0; v < _nvehicles; ++v) {
        _model.add(C[v] <= _I->vcapacity());
    }
}

void CPSolver::genModel2()
{
    //The operations
    _operations = IloIntervalVarArray(_env);
    _opalt = IloIntervalVarArray2(_env, nop());

    for (int i = 0; i < nop(); ++i) {
        string label = "op_" + to_string(i);
        int length = 0;
        if (_I->ispickupnode(i)) {
            length = _I->travellingtime(i, _I->nextnodeid(i));
        }
        IloIntervalVar op(_env, label.c_str());
        op.setEndMax(_I->planhor());
        op.setLengthMin(length);
        op.setLengthMax(length+_I->maxtourlength(_I->groupid(i))- _I->mintourlength(_I->groupid(i)));
        op.setPresent();
        _operations.add(op);

        //The alternatives
        _opalt[i] = IloIntervalVarArray(_env);
        if (_I->ispickupnode(i)) {
            for (int v = 0; v < _nvehicles; ++v) {
                string labelalt = "opa_" + to_string(i) + "_" + to_string(v);
                IloIntervalVar opalt(_env, labelalt.c_str());
                opalt.setOptional();
                _opalt[i].add(opalt);
            }
        }
        
    }

    //The Vehicle Assignment Variables
    _z = IloIntVarArray(_env);
    for (int i = 0; i < nop(); ++i) {
        string label = "z_" + to_string(i);
        _z.add(IloIntVar(_env, 0, _nvehicles-1, label.c_str()));
    }

    //Objective
    IloIntVarArray objexpr(_env);
    for (int i = 0; i < nop(); ++i) if (_I->ispickupnode(i)) objexpr.add(_z[i]);
    //_obj = IloMinimize(_env, IloCountDifferent(objexpr));
    _obj = IloMinimize(_env, 1+IloMax(objexpr));
    _model.add(_obj);


    //Vehicle Assignment
    for (int i = 0; i < nop(); ++i) if (_I->ispickupnode(i)) {
        IloExpr expr(_env);
        for (int v = 0; v < _nvehicles; ++v) expr += v*IloPresenceOf(_env,_opalt[i][v]);
        _model.add(_z[i] == expr);
    }

    //Time Consistency
    for (int i = 0; i < nop(); ++i) if (_I->ispickupnode(i)) {
        for (int j = 0; j < i; ++j) if (i!=j && _I->ispickupnode(j)) {
            IloAnd AndExpr(_env);
            int nexti = _I->nextnodeid(i);
            int nextj = _I->nextnodeid(j);
            AndExpr.add(IloAbs(IloStartOf(_operations[i]) - IloEndOf(_operations[j])) >= _I->travellingtime(i, nextj));
            AndExpr.add(IloAbs(IloEndOf(_operations[i]) - IloStartOf(_operations[j])) >= _I->travellingtime(nexti, j));
            AndExpr.add(IloAbs(IloStartOf(_operations[i])- IloStartOf(_operations[j]))>=_I->travellingtime(i,j));
            AndExpr.add(IloAbs(IloEndOf(_operations[i]) - IloEndOf(_operations[j])) >= _I->travellingtime(nexti, nextj));
            
            _model.add(IloIfThen(_env, _z[i] == _z[j], AndExpr));
        }
    }

    //Operations alternative
    for (int i = 0; i < nop(); ++i) if (_I->ispickupnode(i)) {
        string label = "Alt_" + to_string(i);
        _model.add(IloAlternative(_env, _operations[i], _opalt[i], label.c_str()));
    }

    //Group precedence constraints
    for (int g = 0; g < _I->ngroups(); ++g) {
        int o = _I->firstgroupnode(g);
        while (!_I->islastnode(o)) {
            int nexto = _I->nextnodeid(o);
            if (_I->ispickupnode(o)) {
                assert(_I->isdeliverynode(nexto));
                if (!_I->islastnode(nexto)) {
                    int nextnexto = _I->nextnodeid(nexto);
                    int length = _I->travellingtime(nexto,nextnexto);
                    _model.add(IloEndBeforeStart(_env, _operations[o], _operations[nextnexto], length));
                }
            }
            o = nexto;
        }

        assert(_I->islastnode(o));
        assert(_I->ispickupnode(_I->prevnodeid(o)));
        _model.add(IloEndBeforeStart(_env, _operations[_I->prevnodeid(o)], _operations[_I->firstgroupnode(g)], -_I->maxtourlength(g)));
    }

    //Vehicle Capacity
    IloCumulFunctionExprArray C(_env, _nvehicles);
    for (int v = 0; v < _nvehicles; ++v) C[v] = IloCumulFunctionExpr(_env);
    for (int g = 0; g < _I->ngroups(); ++g) {
        int o = _I->firstgroupnode(g);
        while (!_I->islastnode(o)) {
            int nexto = _I->nextnodeid(o);
            if (_I->ispickupnode(o)) {
                assert(_I->isdeliverynode(nexto));
                for (int v = 0; v < _nvehicles; ++v) {
                    C[v] += IloPulse(_opalt[o][v], _I->groupsize(g));
                }
            }
            o = nexto;
        }
    }
    for (int v = 0; v < _nvehicles; ++v) {
        _model.add(C[v] <= _I->vcapacity());
    }

}


Solution* CPSolver::getSolution()
{
    cout << "CP Status: " << _cp.getStatus() << endl;
    if (_cp.getStatus() == IloAlgorithm::InfeasibleOrUnbounded || _cp.getStatus() == IloAlgorithm::Infeasible  || _cp.getStatus() == IloAlgorithm::Unknown) {
        return nullptr;
    }

    Solution* S = new Solution(_I);
    for (int v = 0; v < _nvehicles; ++v){
        operation* vehicleop = NULL;
        for (int o = 0; o < nop(); ++o) {
            if (_CPModel == CPModel::CP1) {
                if (_cp.isPresent(_opalt[o][v])) {
                    operation* op = S->getop(o);
                    assert(op != NULL);

                    if (vehicleop == NULL) {
                        S->setst(op, _cp.getStart(_opalt[o][v]));
                        vehicleop = op;
                    }
                    else  assert(vehicleop != NULL && S->insertinroute(vehicleop, op, _cp.getStart(_opalt[o][v])));

                }
            }
            else if (_CPModel == CPModel::CP2 || _CPModel == CPModel::CP3) {
                if (_I->ispickupnode(o) && _cp.getValue(_z[o])==v) {
                    operation* pickup = S->getop(o);
                    operation* delivery = S->getop(_I->nextnodeid(o));
                    assert(pickup != NULL && delivery!=NULL);

                    if (vehicleop == NULL) {
                        S->setst(pickup, _cp.getStart(_operations[o]));
                        vehicleop = pickup;
                    }
                    else  assert(vehicleop != NULL && S->insertinroute(vehicleop, pickup, _cp.getStart(_operations[o])));

                    assert(vehicleop != NULL && S->insertinroute(vehicleop, delivery, _cp.getEnd(_operations[o])));
                }
            }
            
        }
    }
    return S;
}
