#include "MILPSolver.h"

void MILPSolver::solvemethod(Solution* S)
{
	try {
		_cplex.setParam(IloCplex::TiLim, _timlim);
		_cplex.setParam(IloCplex::EpGap, _mingap);
		_cplex.setParam(IloCplex::Threads, 1);
		//_cplex.setParam(IloCplex::WorkMem, 5000.0);
		_cplex.solve();
	}

    catch(IloException &e){
        cout << e <<endl;
        exit(1000);
    }
}

MILPSolver::MILPSolver(Instance* I, int OF) : Solver(I,"MILPSolver")
{
	try {
		_model = IloModel(_env); // define modelo
        _cplex = IloCplex(_model);
        _Constraints=IloRangeArray(_env);


		/* ======= Decision variables ======= */
        _y = IloArray< IloNumVarArray >(_env, _I->nnodes());
		for (int i = 0; i < _I->nnodes(); i++) {
			_y[i] = IloNumVarArray(_env);
			for (int j = 0; j < _I->nvehicles(); j++) {
				string label = "Y(" + to_string(i) + "," + to_string(j) + ")";
				if (i == 0 || i == (_I->nnodes()-1) )
					_y[i].add(IloNumVar(_env, 1, 1, ILOBOOL, label.c_str()));
				else
					_y[i].add(IloNumVar(_env, 0, 1, ILOBOOL, label.c_str()));
			}
		}

		_x=IloArray< IloArray< IloNumVarArray > > (_env,_I->nnodes());
		for(int i = 0; i < _I->nnodes(); i++) {
			_x[i] = IloArray< IloNumVarArray >(_env, _I->nnodes());
			for(int j = 0; j < _I->nnodes(); j++) {
				_x[i][j] = IloNumVarArray(_env);
				for(int k=0; k < _I->nvehicles(); k++) {
					string label = "X(" + to_string(i) + ","  + to_string(j) + "," + to_string(k) + ")";
					if (i == j) {
						_x[i][j].add(IloNumVar(_env, 0, 0, ILOBOOL, label.c_str() ));
					} else
						_x[i][j].add(IloNumVar(_env, 0, 1, ILOBOOL, label.c_str() ));
				}
			}
		}

		_u = IloArray< IloNumVarArray >(_env, _I->nnodes());
		for (int i = 0; i < _I->nnodes(); i++) {
			_u[i] = IloNumVarArray(_env);
			for (int j = 0; j < _I->nvehicles(); j++) {
				string label = "U(" + to_string(i) + "," + to_string(j) + ")";
				_u[i].add(IloNumVar(_env, 0, _I->nnodes(), ILOINT, label.c_str()));
			}
		}

		/* ======= Model ======= */

		if (OF == 1) {
			// Objective function Max-Min
			IloNumVar Sm(_env, 0, _I->totalScore(), ILOINT, "Smin");
			_obj=IloMaximize(_env, Sm);
			_model.add(_obj);

			// Linearization
			for (int k = 0; k < _I->nvehicles(); k++) {
				IloExpr score(_env);
				for (int i = 1; i < _I->nnodes() - 1; i++){
					score += (_I->node(i)->score())*_y[i][k];
				}
				string label="c1_"+to_string(k);
				_Constraints.add(IloRange(_env, -IloInfinity, Sm-score, 0, label.c_str()));
			}
		} else {
			// Min-Max
			IloNumVar Dm(_env, 0, _I->totalScore(), ILOINT, "Dmax");
			_obj=IloMinimize(_env, Dm);
			_model.add(_obj);

			double mu = _I->totalScore() / _I->nvehicles();
			for (int k = 0; k < _I->nvehicles(); k++) {
				IloExpr score(_env);
				for (int i = 1; i < _I->nnodes() - 1; i++) {
					score += (_I->node(i)->score())*_y[i][k];
				}

				string label="c1.1_"+to_string(k);
				_Constraints.add(IloRange(_env, 0, Dm-mu+score, IloInfinity, label.c_str()));

				string label2="c1.2_"+to_string(k);
				_Constraints.add(IloRange(_env, 0, Dm+mu-score, IloInfinity, label2.c_str()));
			}
		}

		/* ======= Constraints ======= */

		// m routes
		IloExpr start(_env);
		IloExpr end(_env);
		for (int k = 0; k < _I->nvehicles(); k++) {
			for (int j = 1; j < _I->nnodes(); j++)
				start += _x[0][j][k];
		}
		string label = "c2.1";
		_Constraints.add(IloRange(_env, _I->nvehicles(), start, _I->nvehicles(), label.c_str()));

		for (int k = 0; k < _I->nvehicles(); k++) {
			for (int i = 0; i < _I->nnodes()-1; i++)
				end += _x[i][_I->nnodes()-1][k];
		}
		string label2 = "c2.2";
		_Constraints.add(IloRange(_env, _I->nvehicles(), end, _I->nvehicles(), label2.c_str()));

		// Connectivity
		for (int l = 1; l < _I->nnodes()-1; l++) {
			for (int k = 0; k < _I->nvehicles(); k++) {
				IloExpr conec1(_env);
				IloExpr conec2(_env);
				for (int i = 0; i < _I->nnodes()-1; i++) conec1 += _x[i][l][k];
				for (int j = 1; j < _I->nnodes(); j++) conec2 += _x[l][j][k];

				string label1="c3.1_" +to_string(l) + "_" + to_string(k);
				_Constraints.add(IloRange(_env, 0, conec1 - _y[l][k], 0, label1.c_str() ));

				string label2="c3.2_"+to_string(l) + "_" + to_string(k);
				_Constraints.add(IloRange(_env, 0, conec2 - _y[l][k], 0, label2.c_str() ));
			}
		}

		// At most one visit per node
		for (int i = 1; i < _I->nnodes()-1; i++) {
			IloExpr visit(_env);
			for (int j = 0; j < _I->nvehicles(); j++) visit += _y[i][j];
			string label = "c4_" + to_string(i);
			_Constraints.add(IloRange(_env, -IloInfinity, visit, 1, label.c_str() ));
		}

		// Compatibility
		for (int k = 0; k < _I->nvehicles(); k++) {
			for (int i = 1; i < _I->nnodes()-1; i++) {
				string label = "c5_" + to_string(i) + "_" + to_string(k);
				_Constraints.add(IloRange(_env, -IloInfinity, _y[i][k] - _I->comp(k, i), 0, label.c_str() ));
			}
		}

		// Time budget
		for (int k = 0; k < _I->nvehicles(); k++) {
			IloExpr servTime(_env);
			IloExpr travelTime(_env);
			for (int i = 0; i < _I->nnodes() - 1; i++) {
				Node* O=_I->node(i);
				for (int j = 1; j < _I->nnodes(); j ++) {
					Node* D=_I->node(j);
					travelTime += (_I->distance(O, D))*_x[i][j][k];
				}
				servTime += (_I->node(i)->serviceTime())*_y[i][k];
			}
			string label = "c6_" + to_string(k);
			_Constraints.add(IloRange(_env, -IloInfinity, servTime + travelTime - _I->tmax(), 0, label.c_str() ));
		}

		// Subtour elimination constraints
		for (int k = 0; k < _I->nvehicles(); k++) {
			_u[0][k].setUB(1.0);
			for (int i = 1; i < _I->nnodes(); i++) {

				string label = "c7.1_" + to_string(i) + "_" + to_string(k);
				_Constraints.add(IloRange(_env, -IloInfinity, 2 - _u[i][k], 0, label.c_str() ));

				IloExpr sum(_env);
				for (int j = 0; j < _I->nnodes(); j++) sum += _y[j][k];
				
				string label2 = "c7.2_" + to_string(i) + "_" + to_string(k);
				_Constraints.add(IloRange(_env, -IloInfinity, _u[i][k] - sum, 0, label2.c_str() ));

			}
		}

		for (int i = 1; i < _I->nnodes()-1; i++) {
			for (int j = 1; j < _I->nnodes(); j++) {
				for (int k = 0; k < _I->nvehicles(); k++) {
					if (i!=j) {
						IloExpr expr(_env);
						expr = _u[i][k] - _u[j][k] + 1 - ( _I->nnodes()*(1 - _x[i][j][k]) );
						string label = "c8_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k);
						_Constraints.add(IloRange(_env, -IloInfinity, expr, 0, label.c_str() ));
					}
				}
			}
		}


		// Valid inequality I: upper bound for y_ik
		for (int k = 0; k < _I->nvehicles(); k++) {
			IloNumExpr bound(_env);
			list<int> Ck = _I->compRoute(k);
			int sizeCk = Ck.size();
			for (int j = 1; j < _I->nnodes()-1; j++)
				bound += _y[j][k];
			string label = "c9_" + to_string(k);
			_Constraints.add(IloRange(_env, -IloInfinity, bound - sizeCk, 0, label.c_str() ));
		}

		// Valid inequality II: upper bound for x_ijk
		for (int k = 0; k < _I->nvehicles(); k++) {
			IloNumExpr boundx(_env);

			list<int> Ck = _I->compRoute(k);
			int sizeCk = Ck.size();

			double min_st = 1000000;
			for (int i = 1; i < _I->nnodes(); i++) {
				//Node* O=_I->node(i);
				if( _I->node(i)->serviceTime() < min_st)
					min_st = _I->node(i)->serviceTime();
			}

			double min_tij = 1000000;
			double temp_tij = 0;
			for(list<int>::iterator it = Ck.begin(); it != Ck.end(); it++) {
				list<int>::iterator it2 = it;
				advance(it2, 1);
				Node* O=_I->node(*it);
				while ( it2 != Ck.end() ) {
					Node* D=_I->node(*it2);
					temp_tij = _I->distance(O, D);
					if (temp_tij != 0) {
						if (temp_tij < min_tij)
							min_tij = temp_tij;
					}
					it2++;
				}

				// start
				temp_tij = _I->distance(_I->node(0), O);
				if (temp_tij < min_tij) min_tij = temp_tij;
				//end
				temp_tij = _I->distance(O, _I->node(_I->nnodes()-1));;
				if (temp_tij < min_tij) min_tij = temp_tij;
			}

			int bound = ceil(_I->tmax()/(min_st + min_tij));
			//int finalBound = 0;
			//if (bound < sizeCk) finalBound = bound + 1;
			//else finalBound = sizeCk + 1;
			//cout << sizeCk << " " << bound << " " << finalBound << " " << min(sizeCk,bound) << endl;

			for (int i = 0; i < _I->nnodes()-1; i++) {
				for (int j = 1; j < _I->nnodes(); j++)
					boundx += _x[i][j][k];
			}
			string label = "c10_" + to_string(k);
			_Constraints.add(IloRange(_env, -IloInfinity, boundx - min(sizeCk,bound) - 1, 0, label.c_str() ));
		}

		_model.add(_Constraints);
		//_cplex.exportModel("Model.lp");
	}

    catch (IloException& e) {
        cout << e << endl;
        exit(1000);
    }
}

MILPSolver::~MILPSolver()
{
	//TODO
}

Solution* MILPSolver::recoversolution()
{
	Solution* _Solution = NULL;
	try{
		_Solution = new Solution(_I);
        if (_cplex.getStatus() == IloAlgorithm::Feasible || _cplex.getStatus() == IloAlgorithm::Optimal) {
            for (int k = 0; k < _I->nvehicles(); k++) {
                int i = 0;
                int j = 1;
                while (j != _I->nnodes()) {
                    if (_cplex.getValue(_x[i][j][k]) < 0.5)
                        j++;
                    else if (_cplex.getValue(_x[i][j][k]) > 0.5) {
    					if (i == 0) _Solution->addNode(k,i);
                        _Solution->addNode(k,j);
                        if (j == _I->nnodes() -1) break;
                        i = j;
                        j = 1;
                    }
                }
            }
        }
		/*cout << _cplex.getStatus() << endl;
		cout << gap() << endl;
		cout << value() << endl;*/
	}
    catch (IloException& e) {
        cout << e << endl;
        exit(1000);
    }
	return _Solution;
}
