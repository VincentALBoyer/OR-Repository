#include "CPSolver.h"

void CPSolver::solvemethod(Solution* S)
{
	if (S!= NULL) initSolver(S);
	try {
		_cp.setParameter(IloCP::TimeLimit, _timlim);
		_cp.setParameter(IloCP::OptimalityTolerance, _mingap);
		_cp.setParameter(IloCP::FailLimit, IloIntMax);
		_cp.setParameter(IloCP::Workers, 1);
		_cp.setParameter(IloCP::SearchType, IloCP::Restart);
		//_cp.setParameter(IloCP::RestartFailLimit, 100);
		// poner un cronometro aqui
		/*_cp.startNewSearch();
		while(_cp.next()){
		_cp.out() << "objective value = " << _cp.getObjValue() << endl;
		}
		//_cp.end();*/
		_cp.solve();
	}
    catch(IloException &e){
        cout << e <<endl;
        //exit(1000);
    }
}

CPSolver::CPSolver(Instance* I, int OF, int version) : Solver(I,"CPSolver")
{
    try {

    	//initSolver(S);
        _model=IloModel(_env);
        _cp=IloCP(_model);

        _M=IloTransitionDistance(_env, _I->nnodes());
        _visits = IloIntervalVarArray(_env); // nodes present in the solution
        _visitalts = IloIntervalVarArray2(_env, _I->nnodes()+_I->nvehicles()); // optional visits
        //IloIntArray type(_env,_I->nnodes());

        int DIpos = 0;
        for(int i = 0; i < _I->nnodes(); ++i){
            Node* N = _I->node(i);
            //type[i]=i;
            if(N->isdepot()){ //Initial depot
                string label="DI";
                IloIntervalVar visit(_env,0,0,IloFalse,0,label.c_str());
                _visits.add(visit); // mandatory node
                _visitalts[i]=IloIntervalVarArray(_env);
                DIpos=i;
            }
            else if (i != _I->nnodes() - 1) {
                string label = "c" + to_string(N->index());
                IloIntervalVar visit(_env,label.c_str());
                visit.setOptional(); // optional node
                _visits.add(visit);
                _visitalts[i]=IloIntervalVarArray(_env);

                int size;
                for(int v = 0; v < _I->nvehicles(); ++v){
					string labelalt = label + "_v" + to_string(v);
					size = round(N->serviceTime());
					IloIntervalVar valt(_env,size,size,IloTrue,0,labelalt.c_str());
                	if (_I ->comp(v, i) != 1) valt.setAbsent(); //compatibility
					_visitalts[i].add(valt);
                }
            }

            for (int j = 0; j < _I->nnodes(); ++j){
                int dist=_I->distance(_I->node(i), _I->node(j));
                _M.setValue(i, j, dist);
            }
        }

        int DFpos = _I->nnodes() - 1;
        //type[I->nnodes()-1] = DFpos;
        int i = _I->nnodes() -1 ;
        for(int v = 0; v < _I->nvehicles(); ++v){ //Final depot: one for each vehicle
            string labelf = "DF_" + to_string(v);
            IloIntervalVar visit(_env,0,0,IloFalse,0,labelf.c_str());
            _visits.add(visit); // mandatory node
            _visitalts[i++]=IloIntervalVarArray(_env);
        }

        _R=IloIntervalSequenceVarArray(_env);
        _routespans = IloIntervalVarArray(_env);
        IloCumulFunctionExpr Score(_env);
        for (int v = 0; v < _I->nvehicles(); v++) {
            string label = "r" + to_string(v);
            IloIntervalVarArray a(_env);
            for(int i = 0; i < _I->nnodes() - 1; ++i){
                if (i != DIpos) a.add(_visitalts[i][v]);
                else a.add(_visits[DIpos]);
            }
            a.add(_visits[DFpos+v]); // final depot, one for each route to avoid synchrony
            //_R.add(IloIntervalSequenceVar(_env, a, type,label.c_str()));
            _R.add(IloIntervalSequenceVar(_env, a,label.c_str()));

            string labels = "S_" + to_string(v);
            IloIntervalVar span(_env,labels.c_str());
            _routespans.add(span);

            string labels2 = "SPAN_" + to_string(v);;
            span.setLengthMax( _I -> tmax()); // time budget
            _model.add(IloSpan(_env, span, a, labels2.c_str()));
        }

        //Routes must start and finish at depot
        for (int v = 0; v < _I->nvehicles(); v++) {
            _model.add(IloFirst(_env, _R[v], _visits[DIpos]));
            _model.add(IloLast(_env, _R[v], _visits[DFpos+v]));
        }

        //Traveling distance
        for (int v = 0; v < _I->nvehicles(); v++) {
            string label = "NOL_" + to_string(v);
            _model.add(IloNoOverlap(_env, _R[v], _M, label.c_str()));
        }

        //At most one visit per node
        for (int i = 0; i < _I->nnodes()-1; ++i) if (i != DIpos) {
            string label = "ALT_" + to_string(i);
            _model.add(IloAlternative(_env, _visits[i], _visitalts[i], label.c_str()));
        }

        //Number of vehicles
        if (version != 0) {
        	string label = "V_";
			IloCumulFunctionExpr C(_env);
			for (int i = 0; i < _I->nnodes()-1; ++i) if (i != DIpos) {
				C += IloPulse(_visits[i], 1);
			}
			_model.add(C <= _I->nvehicles()); // == n ??
        }

		// visits bound
		int boundS = _I->totalScore();
        if (version == 0 || version == 2) {
        	list<double> pos_scores; // auxiliary bound-Sm
            int sizeCk = 0;
    		for (int v = 0; v < _I->nvehicles(); v++) {
    			list<int> Ck = _I->compRoute(v);
    			sizeCk = Ck.size();
    			IloNumExpr bound(_env);
    			for (int i = 1; i < _I->nnodes()-1; i++) {
    				bound += IloPresenceOf(_env, _visitalts[i][v]);
    			}
    			_model.add(bound <= sizeCk);
    			pos_scores.push_back(_I->score_comp(v));
    		}
    		pos_scores.sort();
    		boundS = pos_scores.back();
        }

        // Objective
        if (OF == 1) {
            // Linearized objective function Max-Min
            IloNumVar Sm(_env, 0, boundS, ILOINT, "Smin");
    		_obj=IloMaximize(_env, Sm);
    		_model.add(_obj);

    		for (int v = 0; v < _I->nvehicles(); v++) {
            	IloNumExpr totalScore(_env);
    			for (int i = 1; i < _I->nnodes()-1; i++) {
    					totalScore += _I->node(i)->score()*(IloPresenceOf(_env, _visitalts[i][v]));
    			}
    			_model.add(Sm <= totalScore);
            }
        } else if (OF == 2)  {
            // Linearized objective function Min-Max
            IloNumVar Dm(_env, 0, _I->totalScore(), ILOINT, "Dmax");
    		_obj=IloMinimize(_env, Dm);
    		_model.add(_obj);

    		for (int v = 0; v < _I->nvehicles(); v++) {
            	IloNumExpr totalScore(_env);
    			for (int i = 1; i < _I->nnodes()-1; i++) {
    					totalScore += _I->node(i)->score()*(IloPresenceOf(_env, _visitalts[i][v]));
    			}
    			_model.add(Dm >= (_I->totalScore()/_I->nvehicles()) - totalScore);
    			_model.add(Dm >= totalScore - (_I->totalScore()/_I->nvehicles()) );
            }
        } else {
        	// combination of both OF
        	IloNumVar Dm(_env, 0, _I->totalScore(), ILOINT, "Dmax");
			IloNumVar Sm(_env, 0, _I->totalScore(), ILOINT, "Smin");
			_obj=IloMinimize(_env, Dm-Sm);
			_model.add(_obj);

			for (int v = 0; v < _I->nvehicles(); v++) {
				IloNumExpr totalScore(_env);
				for (int i = 1; i < _I->nnodes()-1; i++) {
						totalScore += _I->node(i)->score()*(IloPresenceOf(_env, _visitalts[i][v]));
				}
				_model.add(Dm >= (_I->totalScore()/_I->nvehicles()) - totalScore);
				_model.add(Dm >= totalScore - (_I->totalScore()/_I->nvehicles()) );
				_model.add(Sm <= totalScore);
			}
        }

        //Extracting the model
        //_cp.exportModel("Model.cpo");
    }

    catch (IloException& e) {
    	cout << "Error: " << e << std::endl;
    }
}

CPSolver::~CPSolver()
{
	//TODO
}

Solution* CPSolver::recoversolution()
{
	// Verify the secuence per route
	/*for (int v = 0; v < _I->nvehicles(); ++v) {
		_cp.out() << "V_" + to_string(v) + ":";
		for (IloIntervalVar a = _cp.getFirst(_R[v]); a.getImpl() != 0; a = _cp.getNext(_R[v], a)) {
			_cp.out() << _cp.domain(a) << " ";
			_cp.out() << endl;
		}
		_cp.out() << endl;
	}*/

	Solution *_Solution = new Solution(_I);
	for (int v = 0; v < _I->nvehicles(); ++v) {
		vector<int> present;
		for (int i = 1; i < _I->nnodes()-1; i++) {
			string aux = "c" + to_string(i) + "_v" + to_string(v);
			if (_cp.isPresent(aux.c_str()) == 1) {
				present.push_back(i);
			}
		}
		/*cout << " clientes assigned to vehicle " << v << endl;
		for(int i=0; i < present.size(); i++) cout << present.at(i) << ' ';
		cout << endl;*/

		if (present.size() == 0) { // no clients assigned
			if (_cp.isPresent("DI") == 1) _Solution->addNode(v, 0);
			string finalD = "DF_" + to_string(v);
			if (_cp.isPresent(finalD.c_str()) == 1)	_Solution->addNode(v, _I->nnodes()-1);
		} else {
			for (IloIntervalVar a = _cp.getFirst(_R[v]); a.getImpl() != 0; a = _cp.getNext(_R[v], a)) {

				if (_cp.getEnd(a) == _cp.getEnd(_cp.getFirst(_R[v]))) {
					_Solution->addNode(v, 0);
				}

				for (int i = 1; i < _I->nnodes()-1; i++){
					vector<int>::iterator it;
					it = find(present.begin(), present.end(), i);
					if (it != present.end()) {
						if (_cp.getEnd(_visits[i]) == _cp.getEnd(a) && (*it == i)) {
							present.erase(it);
							_Solution->addNode(v, i);
						}
					}
				}

				if (_cp.getEnd(a) == _cp.getEnd(_cp.getLast(_R[v]))) {
					_Solution->addNode(v, _I->nnodes()-1);
				}
			}
		}
	}

	return _Solution;
}

void CPSolver::initSolver(Solution* S)
{
	assert(S->isFeasible());
	IloSolution IloS(_env);

	for (int k=0; k < _I->nvehicles(); k++) {
		list<int> route = S->getRoute(k);
	    /*for (auto const &i: route) {
	        cout << i << " ";
	    }
	    cout << endl;*/
		int time = 0;
		int prev = 0;
		for(list<int>::iterator it = route.begin(); it != route.end(); it++) {
			//cout << prev << "," << *it << "_r" << k << " " << time << endl;
			if ( *it != 0 && *it != (_I->nnodes()-1) ) {
				IloS.add(_visitalts[*it][k]);
				IloS.setPresent(_visitalts[*it][k]);
				time += _I->distance(_I->node(prev), _I->node(*it));
				IloS.setStart(_visitalts[*it][k], time);
				IloS.setEnd(_visitalts[*it][k], time + (_I->node(*it)->serviceTime()));
				IloS.setStart(_visits[*it], time);
				IloS.setEnd(_visits[*it], time + + (_I->node(*it)->serviceTime()));
				prev = *it;
			}
		}
	}

	_cp.setStartingPoint(IloS);
}
