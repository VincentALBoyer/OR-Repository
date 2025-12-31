#include "LazyMILP.h"

const string LazyMILP::CycleCut = "GSEC";			//GSEC
const string LazyMILP::PathCut = "IPC";				//IPC
const string LazyMILP::SubCycleCut = "SubCycleCut";		//Not Used
const string LazyMILP::CycleCoverCut = "CCC";	//CCC
const string LazyMILP::PathCoverCut = "IPCC";	//IPCC
const string LazyMILP::CondCut = "CondCut";				//Not Used
const string LazyMILP::LogCut = "LC";				//LC
const string LazyMILP::FlowCut = "CC_1";				//CC
const string LazyMILP::InCompCut = "InCompCut";			//IC (Missing) Not Used
const string LazyMILP::InfPathCut = "InfPathCut";		//Not Used: for infeasible path
const string LazyMILP::MatchCut = "MC";			//MC
const string LazyMILP::ScoreCut = "ScoreCut";			//Not Used: cut on floor(z)
const string LazyMILP::SECCut = "CC_2";				//CC (2)
const string LazyMILP::EdgeCut = "EdgeCut";				//Not Used: Cut on the number of nodes (not edges)
const string LazyMILP::ConnectCut = "ConnectCut";		//Not used: joined two sets of nodes
const string LazyMILP::TmaxCut = "ICC";				//ICC


void LazyMILP::solvemethod(Solution* S)
{

	initsolver(S);
	try {
		

		MyCallback CB(this);

		CPXLONG Mode = IloCplex::Callback::Context::Id::Candidate | IloCplex::Callback::Context::Id::Relaxation;
#if USE_CUSTOM_BRANCHING
		Mode |= IloCplex::Callback::Context::Id::Branching;
#endif
		
		_cplex.use(&CB, Mode);
	
		if(_cplex.solve()) _Bound=_cplex.getBestObjValue();

		cout << "Bound: " << _Bound << endl;

		CB.printlog();
		CB.saveLog();

	}
	catch (IloException& ex) {
		cout << "Error Solve Method: " << ex << endl;
	}
}

void LazyMILP::initsolver(Solution* S)
{
	
	//MILPStart
	if (S != NULL) {

		IloNumVarArray Var(_env);
		IloNumArray Val(_env);

		for (int k = 0; k < _I->nvehicles(); ++k) {
			auto R=S->getRoute(k);

            int i = R.front();
			assert(isinitialdepot(i));
			for (auto j : R) if (i != j) {
				assert(isconnected(i, j, k));
				Var.add(_x[getarcid(i,j,k)]); Val.add(1.0);
				if(!isfinaldepot(j)) Var.add(_y[j][k]); Val.add(1);
				i = j;
			}
		}


		_cplex.addMIPStart(Var, Val, IloCplex::MIPStartRepair);
		//_cplex.writeMIPStarts("mipstart.mst");
		Var.end();
		Val.end();
	}

	_cplex.setParam(IloCplex::TiLim, _timlim);
	_cplex.setParam(IloCplex::EpGap, _mingap); 
	_cplex.setParam(IloCplex::Threads, _threads);
	if (STOP_AT_ROOT) _cplex.setParam(IloCplex::NodeLim, 0);
	_cplex.setParam(IloCplex::Param::MIP::Strategy::HeuristicFreq, -1);

	//_cplex.setParam(IloCplex::Param::MIP::Limits::CutsFactor, 0);
	
	//_cplex.setParam(IloCplex::NodeLim, 0);
	//_cplex.setParam(IloCplex::Param::Preprocessing::RepeatPresolve, 0);
	//_cplex.setParam(IloCplex::VarSel, IloCplex::MaxInfeas);
	//_cplex.setParam(IloCplex::NodeSel, IloCplex::BestBound);
	//_cplex.setParam(IloCplex::CutPass, 1);
	//_cplex.setParam(IloCplex::RootAlg, IloCplex::Primal);
	//_cplex.setParam(IloCplex::NodeAlg, IloCplex::Dual);
	//_cplex.setParam(IloCplex::MIPEmphasis, IloCplex::MIPEmphasisOptimality);
	//_cplex.setParam(IloCplex::MIPSearch, IloCplex::Traditional);
	//_cplex.setParam(IloCplex::HeuristicEffort, 0);
	//for (int i = 0; i < _I->nnodes(); i++) for (int k = 0; k < _I->nvehicles(); k++) {
	//	int n = 0;
	//	for (int l = 0; l < _I->nvehicles(); ++l) if (_I->comp(l, i)) n++;
	//	double d = _I->distance(_I->node(initialdepotid()), _I->node(i)) /*+ _I->distance(_I->node(finaldepotid()), _I->node(i))*/;
	//	_cplex.setPriority(_y[i][k], _I->node(i)->score());
	//	for (int j = 0; j < i; j++)
	//		_cplex.setPriority(_x[getarcid(i,j,k)], 0/*(_I->node(i)->score()+ _I->node(j)->score())*/);
	//}

	for (int i = 0; i < _I->nnodes(); i++) for (int k = 0; k < _I->nvehicles(); k++) {
		_cplex.setPriority(_y[i][k], _I->node(i)->score());
		//for (int j = 0; j < i; j++)
		//	_cplex.setpriority(_x[getarcid(i, j, k)], (_i->node(i)->score()+ _i->node(j)->score()));
	}
	
}

void LazyMILP::initmodel()
{
	try {
		_model = IloModel(_env); 
		_cplex = IloCplex(_model);
		_Constraints = IloRangeArray(_env);


		/* ======= Decision variables ======= */
		_y = IloArray< IloNumVarArray >(_env, _I->nnodes());
		for (int i = 0; i < _I->nnodes(); i++) {
			_y[i] = IloNumVarArray(_env);
			for (int k = 0; k < _I->nvehicles(); k++) {
				string label = "Y(" + to_string(i) + "," + to_string(k) + ")";
				if (isdepot(i))
					_y[i].add(IloNumVar(_env, 1, 1, ILOINT, label.c_str()));
				else if (!_I->comp(k, i)) 
					_y[i].add(IloNumVar(_env, 0, 0, ILOINT, label.c_str()));
				else
					_y[i].add(IloNumVar(_env, 0, 1, ILOINT, label.c_str()));
			}
		}

		_x = IloNumVarArray (_env);
		for (int k = 0; k < _I->nvehicles(); k++) {
			for (int i = 0; i < _I->nnodes(); i++) {
				for (int j = 0; j < i; j++) {
					string label = "X(" + to_string(i) + "," + to_string(j) + "," + to_string(k) + ")";
					arc* a = getArc(i, j, k);
					assert(a != NULL);
					assert(a->arcid() == (int)_x.getSize());
					if (!_I->comp(k, i) || !_I->comp(k, j) /*|| (isdepot(i) && isdepot(j))*/) {
						_x.add(IloNumVar(_env, 0, 0, ILOBOOL, label.c_str()));
					}
					else
						_x.add(IloNumVar(_env, 0, 1, ILOBOOL, label.c_str()));
				}
			}
		}

		/* ======= Model ======= */

		if (_OF == 1) {
			// Objective function Max-Min
			_Sm= IloNumVar(_env, 0, floor(_mu), ILOINT, "Smin");
			_obj = IloMaximize(_env, _Sm);
			_model.add(_obj);

			

			// Linearization
			for (int k = 0; k < _I->nvehicles(); k++) {
				IloExpr score(_env);
				for (int i = 0; i < _I->nnodes(); i++) {
					score += (_I->node(i)->score()) * _y[i][k];
				}
				string label = "c1_" + to_string(k);
				_Constraints.add(IloRange(_env, -IloInfinity, _Sm - score, 0, label.c_str()));


				//double maxscore = 0;
				//for (int i = 1; i < _I->nnodes() - 1; i++) if(_I->comp(k,i)) maxscore = fmax(maxscore, _I->node(i)->score());
				//_Constraints.add(IloRange(_env, -IloInfinity, -_Sm - maxscore + score, 0));

				score.end();
			}
			
		}
		else {
			// Min-Max
			
			_Sm= IloNumVar(_env, 0, _mu, ILOFLOAT, "Dmax");
			_obj = IloMinimize(_env, _Sm);
			_model.add(_obj);

			
			for (int k = 0; k < _I->nvehicles(); k++) {
				IloExpr score(_env);
				for (int i = 0; i < _I->nnodes(); i++) {
					score += (_I->node(i)->score()) * _y[i][k];
				}

				string label = "c1.1_" + to_string(k);
				_Constraints.add(IloRange(_env, 0, _Sm - _mu + score, IloInfinity, label.c_str()));
				//_Constraints.add(IloRange(_env, 0, _I->nvehicles()*(_Sm + score) - _I->totalScore(), IloInfinity, label.c_str()));

				string label2 = "c1.2_" + to_string(k);
				_Constraints.add(IloRange(_env, 0, _Sm + _mu - score, IloInfinity, label2.c_str()));
				//_Constraints.add(IloRange(_env, 0, _I->nvehicles() * (_Sm - score) + _I->totalScore(), IloInfinity, label.c_str()));


				score.end();
			}

				
		}

		/* ======= Constraints ======= */

	
		// Connectivity
		for (int k = 0; k < _I->nvehicles(); k++) {
			for (int i = 0; i < _I->nnodes(); i++) {

				IloExpr conec(_env);
				for (int j = 0; j < _I->nnodes(); j++) if (isconnected(i, j, k)) conec += _x[getarcid(i,j,k)];

				string label1 = "c3_" + to_string(i) + "_" + to_string(k);
				int degree = 2;
				if (isdepot(i)) degree = 1;
				_Constraints.add(IloRange(_env, 0, conec - degree*_y[i][k], 0, label1.c_str()));

				conec.end();
			}
		}

		// At most one visit per node
		for (int i = 0; i < _I->nnodes(); i++) if(!isdepot(i)){
			IloExpr visit(_env);
			for (int k = 0; k < _I->nvehicles(); k++) visit += _y[i][k];
			string label = "c4_" + to_string(i);
			_Constraints.add(IloRange(_env, -IloInfinity, visit, 1, label.c_str()));
			visit.end();
		}


		// Time budget
		for (int k = 0; k < _I->nvehicles(); k++) {
			IloExpr travelTime(_env);
			for (int i = 0; i < _I->nnodes(); i++) {
				Node* O = _I->node(i);
				travelTime += O->serviceTime() * _y[i][k];
				for (int j = 0; j < i; j++) if(isconnected(i,j,k)){
					Node* D = _I->node(j);
					assert(fabs(_I->distance(O, D) - _I->distance(D, O))<EPS);
					travelTime += _I->distance(O, D) * _x[getarcid(i,j,k)];
				}
			}
			string label = "c6_" + to_string(k);
			_Constraints.add(IloRange(_env, -IloInfinity, travelTime , _I->tmax(), label.c_str()));
			travelTime.end();
		}

		// Logical cuts
#if PRELOAD_LC 
		for (int k = 0; k < _I->nvehicles(); k++) {
			for (int i : _I->reachablenodes()) if (_I->comp(k, i)) {
				for (int j : _I->reachablenodes()) if (i < j && _I->comp(k, j)) {
					if (!isdepot(i)) _Constraints.add(_x[getarcid(i, j, k)] - _y[i][k] <= 0);
					if (!isdepot(j)) _Constraints.add(_x[getarcid(j, i, k)] - _y[j][k] <= 0);
				}

			}
		}
#endif

		// Checking for infeasible nodes
		Node* S = _I->node(initialdepotid());
		Node* T = _I->node(finaldepotid());
		for (int i = 0; i < _I->nnodes(); i++) if(!isdepot(i)){
			Node* I = _I->node(i);
			for (int j = 0; j < i; j++) if(!isdepot(j)){
				Node* J = _I->node(j);
				double dij = _I->distance(I, J) + I->serviceTime() + J->serviceTime();
				double d1 = _I->distance(S, I) + dij + _I->distance(J, T);
				double d2 = _I->distance(S, J) + dij + _I->distance(I, T);

				if (d1 > _I->tmax() && d2 > _I->tmax()) {
					for (int k = 0; k < _I->nvehicles(); k++) if (isconnected(i, j, k)) {
						_Constraints.add(_y[i][k] + _y[j][k] <= 1);
						_x[getarcid(i, j, k)].setUB(0.0);
					}
				}
				else if (d1 > _I->tmax()) {
					for (int k = 0; k < _I->nvehicles(); k++) if (isconnected(i, j, k)) {
						_Constraints.add(_x[getarcid(S->index(), i, k)] + _x[getarcid(i, j, k)] + _x[getarcid(j, T->index(), k)] <= 2);
					}
				}
				else if (d2 > _I->tmax()) {
					for (int k = 0; k < _I->nvehicles(); k++) if (isconnected(i, j, k)) {
						_Constraints.add(_x[getarcid(S->index(), j, k)] + _x[getarcid(i, j, k)] + _x[getarcid(i, T->index(), k)] <= 2);
					}
				}
				
				if (_I->distance(S, I) + _I->distance(I, T) > _I->tmax())
					for (int k = 0; k < _I->nvehicles(); k++) _y[i][k].setUB(0.0);

				if (_I->distance(S, J) + _I->distance(J, T) > _I->tmax())
					for (int k = 0; k < _I->nvehicles(); k++) _y[j][k].setUB(0.0);

				
			}
		}

		//Arc budget
		for (int k = 0; k < _I->nvehicles(); k++) {
			IloExpr expr(_env);
			for (int i = 0; i < _I->nnodes(); i++) {
				expr -= _y[i][k];
				for (int j = 0; j < i; j++)  {
					expr += _x[getarcid(i, j, k)];
				}
			}
			string label = "AB_" + to_string(k);
			_Constraints.add(IloRange(_env, -1, expr, -1, label.c_str()));
			expr.end();
		}

		//Conflict constraints
		//for (auto p : _I->conflictnodesets()) if(p.size()==2){
		//	
		//	for (int k = 0; k < _I->nvehicles(); k++) {
		//		IloExpr expr(_env);
		//		for (int i : p) {
		//			expr += _y[i][k];
		//		}

		//		_Constraints.add(expr <= 1);

		//		expr.end();
		//	}
		//}

		//For explicit instance
		if (_I->getType() == "explicit") {
			//Fix at 1 xij if yj=1 where j is the duplicate node
			for (int i = 1; i < _I->nnodes()-1; i+=2) {
				int out_idx = i;
				int in_idx = i + 1;
				assert(_I->distance(_I->node(out_idx), _I->node(in_idx)) < EPS);

				IloExpr in_expr(_env);
				IloExpr out_expr(_env);
				for (int k = 0; k < _I->nvehicles(); k++) {
					in_expr += _y[in_idx][k] - _x[getarcid(out_idx, in_idx, k)];
					out_expr += _y[out_idx][k] - _x[getarcid(in_idx, out_idx, k)];
					_Constraints.add(_y[in_idx][k] - _x[getarcid(out_idx, in_idx, k)] == 0);
					_Constraints.add(_y[out_idx][k] - _x[getarcid(out_idx, in_idx, k)] == 0);
				}
				//_Constraints.add(IloRange(_env, 0, in_expr, 0));
				//_Constraints.add(IloRange(_env, 0, out_expr, 0));
				in_expr.end();
				out_expr.end();
			}
			for (int i = 0; i < _I->nnodes(); i++) {
				for (int j = 0; j < i; j++) {
					double dij = _I->distance(_I->node(i), _I->node(j));
					if (dij < _I->tmax()) continue; //skip if the distance is greater than Tmax

					for (int k = 0; k < _I->nvehicles(); k++) {
						if (isconnected(i, j, k)) {
							_x[getarcid(i, j, k)].setUB(0.0);
						}
					}

				}
			}

		}
	


		_model.add(_Constraints);
		//_cplex.exportModel("Model.lp");
	}

	catch (IloException& e) {
		cout << e << endl;
		exit(1000);
	}
}


LazyMILP::LazyMILP(Instance* I, int OF) : Solver(I, "LazyMILP")
{
	_OF = OF;
	_mu = round(100 * (double)(_I->totalScore()) / (double)(_I->nvehicles())) / 100.0;;

	//Undirected Arcs
	_narcs = 0;
	for (int k = 0; k < _I->nvehicles(); ++k) 
		for (int i = 0; i < _I->nnodes(); ++i) 
			for (int j = 0; j < i; ++j) _narcs++;
		
	_Arcs = new arc * [_narcs];
	int aid = 0;
	for (int k = 0; k < _I->nvehicles(); ++k) {
		for (int i = 0; i < _I->nnodes(); ++i) {
			for (int j = 0; j < i; ++j) {
				_Arcs[aid] = new arc(i, j, k, aid);
				assert(getarcid(i, j, k) == aid);
				aid++;
			}
		}
			
	}

	//for (int k = 0; k < _I->nvehicles(); ++k) {
	//	for (int i = 0; i < _I->nnodes(); ++i) {
	//		for (int j = 0; j < i; ++j) {
	//			arc* a = getArc(i, j, k);
	//			auto p = a->getVertices();
	//			assert(i == p.first && j == p.second && k == a->getvid());
	//		}
	//	}
	//}
	
	initmodel();
	
}

LazyMILP::~LazyMILP()
{
	for (int a = 0; a < _narcs; ++a) delete _Arcs[a];
	delete[] _Arcs;
	
}

Solution* LazyMILP::recoversolution()
{
	cout << "CPlex Status: " << _cplex.getStatus() << endl;
	if (_cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded || _cplex.getStatus() == IloAlgorithm::Unknown) {
		//cout<<"MIPSolver did not return a feasible solution"<<endl;
		return nullptr;
	}

	Solution* S = NULL;
	try {
		S = new Solution(_I);
		if (_cplex.getStatus() == IloAlgorithm::Feasible || _cplex.getStatus() == IloAlgorithm::Optimal) {
			for (int k = 0; k < _I->nvehicles(); k++) {
				int h = -1;
				int i = initialdepotid();;
				S->addNode(k, i);
				while (!isfinaldepot(i)) {
					for (int j = 0; j < _I->nnodes(); ++j) if(j!=h){
						if (isconnected(i, j, k) && _cplex.getValue(_x[getarcid(i, j, k)]) > 0.5) {
							S->addNode(k, j);
							h = i;
							i = j;
							break;
						}
					}

				}
			}
		}
		//cout << _cplex.getStatus() << endl;
		cout << gap() << endl;
		cout <<"Cplex Obj: " << value() << " - Min Score: " << S->minScore() << " - Max Dev: " << S->maxDev() << endl;
	}
	catch (IloException& e) {
		cout << e << endl;
		exit(1000);
	}
	return S;
}


double CplexSolution::fitness()
{
	int OF = getOF();
	vector<double> score(_I->nvehicles());
	for (int k = 0; k < _I->nvehicles() ; ++k) {
		vector<int> R = getRoutes(k).front();

		score[k] = fitness(R);
	}
	auto it = score.begin();
	if (OF == 1) {
		it=min_element(score.begin(), score.end());
	}
	else if (OF == 2) {
		it = max_element(score.begin(), score.end());
	}
	else return 0.0;

	return *it;
}

double CplexSolution::fitness(vector<int>& R) {
	double score = this->score(R);

	if (getOF() == 1) return score;
	else if (getOF() == 2) 
		return fabs(score - _LMILP->_mu);
	else return 0.0;
}

double CplexSolution::fitness(vector<vector<int>> &Routes)
{
	int OF = getOF();
	vector<double> score(_I->nvehicles());
	int k = 0;
	for (auto &R : Routes) {
		score[k++] = fitness(R);
	}
	auto it = score.begin();
	if (OF == 1) {
		it = min_element(score.begin(), score.end());
	}
	else if (OF == 2) {
		it = max_element(score.begin(), score.end());
	}
	else return 0.0;

	return *it;
}




vector<int> CplexSolution::getRandomPath(int k, int s, double threshold)
{
	int n = _I->nnodes();

	vector<int> candidates;
	vector<double> eval(_LMILP->_narcs, 0.0);
	for (int a = 0; a < _LMILP->_narcs; ++a) {
		double w = getXVal(a);
		arc* A = _LMILP->_Arcs[a];
		if (w > threshold && A->getvid()==k && (s < 0 || A->isConnectedTo(s))) {
			candidates.push_back(a);
			eval[a] = w;
			assert(_I->comp(k, A->geti()));
			assert(_I->comp(k, A->getj()));

		}
	}
	vector<int> Path;
	if (!candidates.empty()) {
		double alpha = 0.5;
		auto M = minmax_element(candidates.begin(), candidates.end(), [&](int i, int j) {return  eval[i] < eval[j]; });
		//sort(candidates.begin(), candidates.end());
		double T = eval[*M.second] - alpha * (eval[*M.second] - eval[*M.first]);
		auto it = remove_if(candidates.begin(), candidates.end(), [&](int i) {return  eval[i] < T; });
		int aid = *RouletteWheelSelection(candidates.begin(), it/*candidates.end()*/, [&](int i) {return  eval[i];  });
		Path.push_back(_LMILP->_Arcs[aid]->geti());
		Path.push_back(_LMILP->_Arcs[aid]->getj());
	}
	else {
		Path.push_back(initialdepotid());
		if (!isinitialdepot(s)) Path.push_back(s);
	}


	extendRandomPath(Path, k, threshold);
	reverse(Path.begin(),Path.end());
	extendRandomPath(Path, k, threshold);

	if (isinitialdepot(Path.back()) || isfinaldepot(Path.front())) reverse(Path.begin(), Path.end());

	if (isinitialdepot(Path.front()) && !isfinaldepot(Path.back())) Path.push_back(finaldepotid());
	else if (!isinitialdepot(Path.front()) && isfinaldepot(Path.back())) Path.insert(Path.begin(),initialdepotid());
	else if (!isinitialdepot(Path.front()) && !isfinaldepot(Path.back()) ){
		Node* S = _I->node(initialdepotid());
		Node* T = _I->node(finaldepotid());
		Node* B = _I->node(Path.back());
		Node* F = _I->node(Path.front());
		double d_st = _I->distance(S, F) + _I->distance(B, T);
		double d_ts = _I->distance(B, S) + _I->distance(T, F);
		if(d_ts<d_st) reverse(Path.begin(), Path.end());
		Path.insert(Path.begin(), initialdepotid());
		Path.push_back(finaldepotid());
	}

	assert(!Path.empty());

	return Path;

}

vector<int> CplexSolution::getRandomInfPath(int s, int ks)
{
	double** X = NULL;
	if (ks < 0) X = extractCompleteNetwork();
	else X = extractVehicleNetwork(ks);

	double l = _I->node(s)->serviceTime();
	double sum = 0.0;
	vector<int> Path;
	Path.push_back(s);

	vector<bool> _RStatus(_nnodes, false);
	_RStatus[s] = true;
	_RStatus[initialdepotid()] = true;
	_RStatus[finaldepotid()] = true;
	do {
		int i = Path.back();
		_RStatus[i] = true;

		vector<int> Candidates;
		for (int j = 0; j < _nnodes; ++j) if (!_RStatus[j] && X[i][j] >=EPS && sum + X[i][j] >= Path.size() - 1)
			Candidates.push_back(j);


		int j = finaldepotid();
		if (!Candidates.empty()) {
			j = *RouletteWheelSelection(Candidates.begin(), Candidates.end(), [&](int i) {return  round(X[i][j] * 100);  });
			Path.push_back(j);
			sum += X[i][j];
			l += _I->distance(_I->node(i), _I->node(j)) + _I->node(j)->serviceTime();
		}
		else {
			break;
		}
	} while (l <= _I->tmax());

	assert (!isdepot(Path.front()));
	assert (!isdepot(Path.back()));
	
	//We manually add the depot to minimize the total time
	Node* S = _I->node(initialdepotid());
	Node* T = _I->node(finaldepotid());
	Node* B = _I->node(Path.back());
	Node* F = _I->node(Path.front());
	double d_st = _I->distance(S, F) + _I->distance(B, T);
	double d_ts = _I->distance(B, S) + _I->distance(T, F);
	if (d_ts < d_st) reverse(Path.begin(), Path.end());
	Path.insert(Path.begin(), initialdepotid());
	Path.push_back(finaldepotid());
	


	for (int i = 0; i < _I->nnodes(); ++i) delete[] X[i];
	delete[] X;
	return Path;
}

vector< vector<int> > CplexSolution::getHeuristicRoutes()
{
	vector< vector<int> > Routes(_I->nvehicles());
	for (int k = 0; k < _I->nvehicles(); ++k)
		Routes[k] = getRandomPath(k/*, initialdepotid()*/);
	return Routes;

	//IloArray< IloNumArray > YVal(_LMILP->_env, _I->nvehicles());
	//for (int k = 0; k < _I->nvehicles(); ++k) YVal[k] = getYVal(k);

	//vector< vector<int> > Routes(_I->nvehicles());
	//vector<int> vids;
	//vector<double> score(_I->nvehicles(),0.0);
	//for (int k = 0; k < _I->nvehicles(); ++k) {
	//	vids.push_back(k);
	//	Routes[k].push_back(initialdepotid());
	//	Routes[k].push_back(finaldepotid());
	//}

	//vector<int> nodes;
	//for (int i = 0; i < _I->nnodes(); ++i) if (!isdepot(i)) nodes.push_back(i);

	//double mu = getmu();
	//while (!vids.empty()) {
	//	auto itv = vids.end();
	//	itv = min_element(vids.begin(), vids.end(), [&](int k, int l) {
	//		if (getOF() == 1) return score[k] < score[l];
	//		else return fabs(score[k] - mu) > fabs(score[l] - mu);
	//		});
	//	int k = *itv;

	//	vector<int> Candidate;
	//	copy_if(nodes.begin(), nodes.end(), back_inserter(Candidate), [YVal, k](int i) {return YVal[k][i] > EPS; });

	//	int iselected = -1;
	//	while (!Candidate.empty() && iselected<0) {
	//		auto it = RouletteWheelSelection(Candidate.begin(), Candidate.end(), [YVal, k](int i) {return YVal[k][i] * 100; });
	//		int i = *it;
	//		double sc = _I->node(i)->score();
	//		if (_I->comp(k, i))
	//			if (getOF() == 1 || fabs(score[k] + sc - mu) < fabs(score[k] - mu))
	//				if (bestinsert(Routes[k], i)) {
	//					iselected = i;
	//				}
	//		Candidate.erase(it);
	//	}

	//	if (iselected>=0) {
	//		score[k] += _I->node(iselected)->score();
	//		for (int l = 0; l < _I->nvehicles(); ++l) YVal[l][iselected] = -1;
	//	}
	//	else vids.erase(itv);
	//	
	//	//sort(Candidate.begin(), Candidate.end(), [YVal, k](int i, int j) {return YVal[k][i] > YVal[k][j]; });
	//	//auto it = Candidate.begin();
	//	//while(it != Candidate.end()) {
	//	//	int i = *it;
	//	//	double sc = _I->node(i)->score();
	//	//	if(_I->comp(k,i))
	//	//		if (getOF() == 1 || fabs(score[k] + sc - mu) < fabs(score[k] - mu)) 
	//	//			if (bestinsert(Routes[k], i)) 
	//	//				break;
	//	//		
	//	//	it++;
	//	//}

	//	//if (it != Candidate.end()) {
	//	//	int i = *it;
	//	//	score[k] += _I->node(i)->score();
	//	//	for (int l = 0; l < _I->nvehicles(); ++l) YVal[l][i] = 0.0;
	//	//}
	//	//else vids.erase(itv);
	//}
	//
	////cout <<"Fitness: " << fitness(Routes) << endl;
	//YVal.end();
	//return Routes;
}


vector<int> CplexSolution::getMegaCycle(int k, double threshold)
{
	int n = _I->nnodes();
	vector<bool> _RStatus(_nnodes, false);
	for (int i = 0; i < n; ++i) if (!isdepot(i)) {
		double eval = 0;
		for (int j = 0; j < n; ++j) if (!isinitialdepot(j) && (!_I->comp(k,j) || getXVal(i, j, k) > EPS)) {
			_RStatus[i] = true;
			break;
		}
		
	}

	stack<int> stack;
	stack.push(initialdepotid());
	_RStatus[initialdepotid()] = true;
	_RStatus[finaldepotid()] = true;

	while (!stack.empty()) {
		int i = stack.top();
		stack.pop();

		for (int j = 0; j < n; ++j) if (!_RStatus[j] && getXVal(i, j, k) > threshold) {
			_RStatus[j] = true;
			stack.push(j);
		}
	}

	vector<int> cycle;
	for (int i = 0; i < n; ++i) if (!_RStatus[i]) cycle.push_back(i);
		
	return cycle;
}

vector<int> CplexSolution::getCycle(int s) {
	vector<int> Path;

	int n = _I->nnodes();

	vector<int> P(n, -2);
	
	stack<int> Q;
	Q.push(s);
	//_RStatus[s] = true;
	P[s] = -1;

	while (!Q.empty()) {
		int i = Q.top();
		Q.pop();		
		for (int j = 0; j < n; ++j) if (j != P[i] && P[j] < 0 && hasArc(i, j) && !isdepot(j)) {
			Q.push(j);
			P[j] = i;
			if (j==s) {
				//We get the path
				int k = P[s];
				while (k != s) {
					assert(P[k] >= 0);
					Path.push_back(k);
					k = P[k];
				}
				Path.push_back(k);
				assert(Path.size() > 1 && k == s);
				return Path;
				//break;
			}
			break;
		}
	}

	return Path;


}

vector<int> CplexSolution::bfs(double** RNetwork, int n, int s, int t, double threshold) {

	vector<int> S; S.push_back(s);
	vector<int> T;
	if (t >= 0) T.push_back(t);
	else {
		T.push_back(initialdepotid());
		T.push_back(finaldepotid());
	}
	return bfs(RNetwork, n, S, T, threshold);

}

vector<int> CplexSolution::bfs(double** RNetwork, int n, vector<int>& S, vector<int>& T, double threshold)
{
	vector<int> Path;

	assert(RNetwork != NULL);
	vector<int> P(n, -2);

	for (auto t : T) P[t] = -3;

	stack<int> Q;
	for (auto s : S) {
		Q.push(s);
		P[s] = -1;
	}
	

	while (!Q.empty()) {
		int i = Q.top();
		Q.pop();
		for (int j = 0; j < n; ++j) if (P[j]<=-2 && RNetwork[i][j] > threshold) {
			Q.push(j);
			bool targetfound = (P[j] == -3);
			P[j] = i;
			if (targetfound) {
				
				for (int k = j; k >= 0; k = P[k]) Path.push_back(k);

				assert(Path.size() > 1 && P[Path.back()] == -1);
				//assert(find(T.begin(), T.end(), Path.front()) != T.end());
				return Path;
			}
		}
	}


	return Path;
}

pair< vector<int>, vector<int> > CplexSolution::getShoresMinCut(int s, int t, int k)
{
	vector<int> S; S.push_back(s);
	vector<int> T; 
	if(t>=0) T.push_back(t);
	else {
		T.push_back(initialdepotid());
		T.push_back(finaldepotid());
	}
	return getShoresMinCut(S, T, k);

}

pair<vector<int>, vector<int>> CplexSolution::getShoresMinCut(vector<int>& S, vector<int>& T, int k)
{
	int n = _I->nnodes();
	double** RNetwork = NULL;
	if (k < 0) RNetwork = extractCompleteNetwork();
	else RNetwork = extractVehicleNetwork(k);

	double maxflow = 0.0;
	vector<int> path = bfs(RNetwork, n, S, T, EPS);
	while (!path.empty()) {
		double pathflow = DBL_MAX;;
		int i = path.front();
		for (auto j : path) if (i != j) {
			pathflow = fmin(pathflow, RNetwork[j][i]);
			i = j;
		}

		assert(pathflow > EPS);

		//Warning Path is in reversed order
		i = path.front();
		for (auto j : path) if (i != j) {
			RNetwork[j][i] -= pathflow;
			RNetwork[i][j] += pathflow;
			i = j;
		}



		maxflow += pathflow;

		path = bfs(RNetwork, n, S, T);

	}

	//cout << "MaxFlow: " << maxflow << endl;

	//Then we get the shores
	vector<int> shore1; // Reachable shore
	vector<int> shore2; // Unreachable shore

	vector<bool> _RStatus(_nnodes, false);
	vector<int> Q;
	for (auto s : S) {
		Q.push_back(s);
		_RStatus[s] = true;
	}

	while (!Q.empty()) {
		int i = Q.back();
		Q.pop_back();
		for (int j = 0; j < n; ++j) if (!_RStatus[j] && RNetwork[i][j] > EPS) {
			Q.push_back(j);
			_RStatus[j] = true;
		}
	}

	for (int i = 0; i < _nnodes; ++i) {
		if (_RStatus[i])  shore1.push_back(i); // Node is reachable, add to shore1
		else/* if (k < 0 || _I->comp(k, i))*/ shore2.push_back(i); // Node is not reachable, add to shore2
	}

	double flow = getFlowValue(shore1, shore2);
	//assert(fabs(maxflow - flow) < 0.1-EPS);


	for (int i = 0; i < n; ++i) delete[] RNetwork[i];
	delete[] RNetwork;


	return pair< vector<int>, vector<int> >(shore1, shore2);
}

vector<int> CplexSolution::getRoute(int k) {
	vector<int> R;
	int j = finaldepotid();
	R.push_back(j);
	while (j != initialdepotid()) {
		for (int i = 0; i < _I->nnodes(); ++i) if (i != j)
			if (getXVal(i, j, k) >= 0.5) {
				//assert(i == _S->getBestPrev(j));
				j = i;
				break;
			}
		if (j == finaldepotid()) j = initialdepotid();
		assert(find(R.begin(), R.end(), j) == R.end());

		R.push_back(j);
	}
	assert(j == 0);
	//Warning: The route is returned in reverse order
	return R;
}

vector<vector<int>> CplexSolution::getRoutes(int k) {
	vector< vector<int> > Routes;

	vector<int> sources;
	for (int s = 0; s < _I->nnodes() - 1; ++s) if (!isinitialdepot(s))
		if (getXVal(initialdepotid(), s, k) > EPS) sources.push_back(s);

	if (sources.empty()) {
		vector<int> R;
		R.push_back(initialdepotid());
		R.push_back(finaldepotid());
		Routes.push_back(R);
		return Routes;
	}

	for (auto s : sources) {
		vector<int> R;
		R.push_back(initialdepotid());
		R.push_back(s);
		int h = initialdepotid();
		int i = s;
		while (!isfinaldepot(i)) {
			for (int j = 0; j < _I->nnodes(); ++j) if(j!=h)
				if (getXVal(i, j, k) > EPS) {
					h = i;
					i = j;
					break;
				}
			if (i == initialdepotid()) {
				R.clear();
				break;
			}
			assert(find(R.begin(), R.end(), i) == R.end());

			R.push_back(i);
			
		}
		if(!R.empty()) Routes.push_back(R);
	}

	if (Routes.empty()) {
		vector<int> R;
		R.push_back(initialdepotid());
		R.push_back(finaldepotid());
		Routes.push_back(R);
	}

	if (getOF() == 1) {
		sort(Routes.begin(), Routes.end(), [&](vector<int>& R1, vector<int>& R2) {return fitness(R1) > fitness(R2); });
	}
	else if (getOF() == 2) {
		sort(Routes.begin(), Routes.end(), [&](vector<int>& R1, vector<int>& R2) {return fitness(R1) < fitness(R2); });
	}

	return Routes;
}

vector<int> CplexSolution::repairroute(const vector<int>& R, int k, double MinScore) {

	assert(R.empty() || (isinitialdepot(R.front()) && isfinaldepot(R.back())));
	vector<int> r = R;
	//for (auto i : r) cout << i << " "; cout << endl;
	//First we remove all incompatible nodes
	auto it = r.begin();
	while (it != r.end()) {
		if (_I->comp(k, *it))
			it++;
		else
			it = r.erase(it);
	}

	//cout<<"T:"; for (auto i : r) cout << i << " "; cout << endl;

	//Then we try to reduce the route length
	auto rdata = getRouteData(r);
	double* eval = new double[_nnodes];
	//cout << rdata.first<<" "<<rdata.second << endl;
	//cout << _I->tmax() << endl;
	while (rdata.first > _I->tmax() && rdata.second > MinScore) {
		for (auto it = r.begin(); it != r.end(); it++) {
			if (it == r.begin() || it + 1 == r.end()) eval[*it] = -INFINITY;
			else {
				int i = min(*(it - 1), _I->nnodes() - 1);
				int j = *it;
				int k = *(it + 1);
				double dij = _I->distance(_I->node(i), _I->node(j));
				double djk = _I->distance(_I->node(j), _I->node(k));
				double dik = _I->distance(_I->node(i), _I->node(k));
				//assert(_I->node(j)->score() > EPS);
				eval[j] = (dij + djk - dik + _I->node(j)->serviceTime()) / (double)_I->node(j)->score();
				//eval[j] = 1.0 / (double)_I->node(j)->score();
			}
		}
		auto bestit = max_element(r.begin(), r.end(), [&](int i, int j) {return eval[i] < eval[j]; });
		r.erase(bestit);
		rdata = getRouteData(r);
	}
	delete[] eval;
	//for (auto i : r) cout << i << " "; cout << endl;
	//cout<<rdata.second<<endl;
	//cout<<MinScore<<endl;
	if (rdata.second < MinScore - EPS) r.clear();
	//for (auto i : r) cout << i << " "; cout << endl;
	assert(r.empty() || (isinitialdepot(r.front()) && isfinaldepot(r.back())));

	return r;
}

void CplexSolution::removeDuplicateNodes(vector<vector<int>>& Routes)
{
	//for (int k = 0; k < _I->nvehicles(); ++k) {
	//	auto it = Routes[k].begin();
	//	while (it != Routes[k].end()) {
	//		if (!_I->comp(k, *it)) 
	//			it = Routes[k].erase(it);
	//		else it++;
	//	}
	//}

	vector<int> VisitedNodes;
	for (int k = 0; k < _I->nvehicles(); ++k)
		for (auto i : Routes[k])  VisitedNodes.push_back(i);

	vector<int> sortedNodes;
	copy_if(VisitedNodes.begin(), VisitedNodes.end(), std::back_inserter(sortedNodes), [&](int i) {return !isdepot(i); });
	sort(sortedNodes.begin(), sortedNodes.end());
	auto last = unique(sortedNodes.begin(), sortedNodes.end());
	sortedNodes.erase(last, sortedNodes.end());

	sort(sortedNodes.begin(), sortedNodes.end(), [&](int i, int j) {return _I->node(i)->score() > _I->node(j)->score(); });

	vector< vector<int>::iterator > iti(_I->nvehicles());
	vector<double> score(_I->nvehicles());
	for (int k = 0; k < _I->nvehicles(); ++k) score[k] = this->score(Routes[k]);

	for (int i : sortedNodes) {
		auto n = count(VisitedNodes.begin(), VisitedNodes.end(), i);

		vector<int> vids;
		if (n > 1) {
			for (int k = 0; k < _I->nvehicles(); ++k) {
				iti[k] = find(Routes[k].begin(), Routes[k].end(), i);
				if (iti[k] != Routes[k].end())  vids.push_back(k);
			}
			assert(vids.size() == n);
		}

		while (vids.size() > 1) {

			auto itv = vids.end();
			itv = max_element(vids.begin(), vids.end(), [score](int vi, int vj) {return score[vi] < score[vj]; });

			assert(itv != vids.end());

			Routes[*itv].erase(iti[*itv]);
			score[*itv] -= _I->node(i)->score();
			vids.erase(itv);

		}
	}


}

void CplexSolution::adjustRoutesLength(vector<vector<int>>& Routes)
{
	double* eval = new double[_nnodes];
	for (auto& r : Routes) {
		
		//We try to reduce the route length
		double length = this->length(r);
		
		//cout << rdata.first<<" "<<rdata.second << endl;
		//cout << _I->tmax() << endl;
		while (length > _I->tmax() /*- EPS*/) {
			for (auto it = r.begin(); it != r.end(); it++) {
				if (it == r.begin() || it + 1 == r.end()) eval[*it] = -INFINITY;
				else {
					int i = min(*(it - 1), _I->nnodes() - 1);
					int j = *it;
					int k = *(it + 1);
					double dij = _I->distance(_I->node(i), _I->node(j));
					double djk = _I->distance(_I->node(j), _I->node(k));
					double dik = _I->distance(_I->node(i), _I->node(k));
					eval[j] = (dij + djk - dik + _I->node(j)->serviceTime()) ;
				}
			}
			auto bestit = max_element(r.begin(), r.end(), [&](int i, int j) {return eval[i] / (double)_I->node(i)->score() < eval[j] / (double)_I->node(j)->score(); });
			length -= eval[*bestit];
			r.erase(bestit);
		}
		
		assert(r.empty() || (isinitialdepot(r.front()) && isfinaldepot(r.back())));
	}
	delete[] eval;
}

void CplexSolution::optimizeRoutesLength(vector<vector<int>>& Routes)
{
	for (auto& R : Routes) {
		bool improved = true;
		while (improved) {
			improved = false;
			double length = this->length(R);;
			for (auto iti = R.begin(); iti != R.end() && !improved; ++iti) if(!isdepot(*iti)) {
				for (auto itj = iti+1; itj != R.end() && !improved; ++itj) if(!isdepot(*itj)) {
					std::swap(*iti,*itj);
					double newLength = this->length(R);
					if (newLength < length) {
						length = newLength;
						improved = true;
					}
					else {
						// Swap back if no improvement
						std::swap(*iti, *itj);
					}
				}
			}
		}

		//vector<int> Node = R;
		//auto last = remove_if(Node.begin(), Node.end(), [&](int i) {return isdepot(i); });
		//Node.erase(last, Node.end());
		//sort(Node.begin(), Node.end(), [&](int i, int j) {return _I->node(i)->score() > _I->node(j)->score(); });
		//for (auto i : Node) {
		//	auto it = find(R.begin(), R.end(), i);
		//	assert(it != R.end());
		//	R.erase(it);
		//	assert(bestinsert(R, i));
		//}

	}
}

void CplexSolution::improveRoutes(vector<vector<int>>& Routes, double target)
{
	vector<double> score(_I->nvehicles());
	for (int k = 0; k < _I->nvehicles(); ++k) score[k] = this->score(Routes[k]);
	//A small routine to improve the solution deviation
	if (getOF() == 2) {
		double mu = getmu();

		for (int k = 0; k < _I->nvehicles(); ++k) if (Routes[k].size() > 2 && score[k] >= mu) {
			vector<int> nodes = Routes[k];
			auto last = remove_if(nodes.begin(), nodes.end(), [&](int i) {return isdepot(i); });
			nodes.erase(last, nodes.end());
			sort(nodes.begin(), nodes.end(), [&](int i, int j) {return _I->node(i)->score() > _I->node(j)->score(); });

			for (auto i : nodes) if (fabs(score[k] - mu) > fabs(score[k] - _I->node(i)->score() - mu)) {
				auto it = find(Routes[k].begin(), Routes[k].end(), i);
				assert(it != Routes[k].end());
				Routes[k].erase(it);
				score[k] -= _I->node(i)->score();

			}
		}
	}

	//A small routine to balance the load
	if (getOF() == 1) {

		double minscore = *min_element(score.begin(), score.end());
		minscore = fmax(minscore, target);

		vector<int> nvcomp(_I->nnodes(), 0);
		for (int i = 0; i < _I->nnodes(); ++i) for (int k = 0; k < _I->nvehicles(); ++k) if (_I->comp(k, i)) nvcomp[i]++;

		for (int k = 0; k < _I->nvehicles(); ++k) if (Routes[k].size() > 2 && score[k] > minscore) {
			vector<int> nodes(Routes[k]);
			//auto last = remove_if(nodes.begin(), nodes.end(), [&](int i) {return isdepot(i); });
			//nodes.erase(last, nodes.end());
			
			sort(nodes.begin(), nodes.end(), [&](int i, int j) {return /*nvcomp[i] * */_I->node(i)->score() > /*nvcomp[j] * */_I->node(j)->score(); });

			for (auto i : nodes) if (!isdepot(i) && score[k] - _I->node(i)->score() > minscore && nvcomp[i] > 1) {
				auto it = find(Routes[k].begin(), Routes[k].end(), i);
				assert(it != Routes[k].end());
				Routes[k].erase(it);
				score[k] -= _I->node(i)->score();

			}
		}

	}

	//optimizeRoutesLength(Routes);
	
	vector<double> length(_I->nvehicles());
	vector<int> vids;
	for (int k = 0; k < _I->nvehicles(); ++k) {
		length[k] = this->length(Routes[k]);
		vids.push_back(k);
	}
	sort(vids.begin(), vids.end(), [&](int k, int l) { return score[k] < score[l]; });
	
	vector< bool > visited(_I->nnodes(), false);
	for (const auto& route : Routes)  for (int i : route) visited[i] = true;
	vector<int> S;
	for (int i = 0; i < _I->nnodes(); ++i) if (!visited[i]) S.push_back(i);
	sort(S.begin(), S.end(), [&](int i, int j) { return _I->node(i)->score() > _I->node(j)->score(); });

	double mu = getmu();
	while (true) {
		auto itv = vids.end();
		if (getOF() == 1) itv = min_element(vids.begin(), vids.end(), [score](int k, int l) {return score[k] < score[l]; });
		else if (getOF() == 2) itv = max_element(vids.begin(), vids.end(), [score,mu](int k, int l) {return fabs(mu-score[k]) < fabs(mu-score[l]); });

		auto it = S.begin();
		while (it != S.end()) {
			int j = *it;
			double nodescore = _I->node(j)->score();
			if ((getOF() == 1 || fabs(score[*itv] + nodescore - mu) < fabs(score[*itv] - mu)) && _I->comp(*itv, j)) {

				if (bestinsert(Routes[*itv], j)/* || swapinsert(Routes[*itv], j)!=j*/) {
					//assert(find(Routes[*itv].begin(), Routes[*itv].end(), j) != Routes[*itv].end());
					length[*itv] = this->length(Routes[(*itv)]);
					assert(length[*itv] <= _I->tmax());
					score[*itv] += nodescore;
					break;
				}
			}
			
			it++;
		}
		if (it == S.end()) break;
		else S.erase(it);
	}

}

void CplexSolution::reorderRoutes(vector<vector<int>>& Routes)
{
	vector<double> scores(_I->nvehicles(), 0.0);
	for (int k = 0; k < _I->nvehicles(); k++) scores[k] = score(Routes[k]);

	for (int k = 0; k < _I->nvehicles(); k++) {
		for (int l = k + 1; l < _I->nvehicles(); l++) if (scores[k] > scores[l]) {
			bool kcandol = true;
			for (int i : Routes[l]) if (!_I->comp(k, i)) {
				kcandol = false;
				break;
			}

			bool lcandok = true;
			for (int i : Routes[k]) if (!_I->comp(l, i)) {
				lcandok = false;
				break;
			}

			if (kcandol && lcandok) {
				swap(Routes[k], Routes[l]);
				swap(scores[k], scores[l]);
			}
		}

	}
}

vector<int> CplexSolution::longestPath(int source, int k)
{
	vector<double> dist(_nnodes, -DBL_MAX);
	dist[source] = 0.0;

	// Vector to store predecessors
	vector<int> pred(_nnodes, -1);

	vector<int> Q;
	for (int i = 0; i < _nnodes; i++) Q.push_back(i);

	// Process vertices in topological order
	while(!Q.empty()) {
		auto it = max_element(Q.begin(), Q.end(), [&](int i, int j) {return dist[i] < dist[j]; });
		int u = *it;
		Q.erase(it);
		if (dist[u] <= -DBL_MAX)
			break;;
		
		for (int i = 0; i < _nnodes; i++) {
			double val = getXVal(i, u, k);
			if (val != 0 && dist[i] < dist[u] + val) {
				dist[i] = dist[u] + val;
				pred[i] = u;
			}
		}
		
	}

	return pred;
	
}

pair<IloNumVarArray*, IloNumArray*> CplexSolution::extractSolution()
{
	IloNumVarArray* X = new IloNumVarArray(_LMILP->_env);
	IloNumArray* XVal = new IloNumArray(_LMILP->_env);

	

	double F = fitness();

	vector< unordered_set<int> > VisitedNodes(_I->nvehicles());

	X->add(_LMILP->_Sm); XVal->add(F);

	for (int aid = 0; aid < _LMILP->_narcs; ++aid) {
		arc* a = _LMILP->_Arcs[aid];
		IloNum val = _XVal[aid];// (getXVal(a->geti(), a->getj(), a->getvid()) > 0.5) ? 1.0 : 0.0;
		X->add(_LMILP->_x[a->arcid()]);
		XVal->add(val);
		if (val > 0.5 ) {
			VisitedNodes[a->getvid()].insert(a->geti());
			VisitedNodes[a->getvid()].insert(a->getj());
		}
	}


	for (int i = 0; i < nnodes(); ++i) /*if(!isdepot(i))*/{
		for (int k = 0; k < _I->nvehicles(); ++k) {
			IloNum val = 0;
			if (find(VisitedNodes[k].begin(), VisitedNodes[k].end(), i) != VisitedNodes[k].end()) {
				val = 1;
				//assert(_I->comp(k, i));
			}
			X->add(_LMILP->_y[i][k]); XVal->add(val);
		}
	}

	VisitedNodes.clear();

	std::pair< IloNumVarArray*, IloNumArray* > P(X, XVal);

	//Solution *Sol = new Solution(_I);
	//
	//for (int k = 0; k < _I->nvehicles(); k++) {
	//	int h = -1;
	//	int i = initialdepotid();;
	//	Sol->addNode(k, i);
	//	while (!isfinaldepot(i)) {
	//		for (int j = 0; j < _I->nnodes(); ++j) if (j != h) {
	//			if (_LMILP->isconnected(i, j, k) && _XVal[_LMILP->getarcid(i, j, k)] > 0.5) {
	//				Sol->addNode(k, j);
	//				h = i;
	//				i = j;
	//				break;
	//			}
	//		}

	//	}
	//}
	//assert(Sol->isFeasible());
	//delete Sol;
	

	return P;
}


void MyCallback::addLazyCuts(const IloCplex::Callback::Context& context)
{
	if (!context.inCandidate()) return;

	try {

		CplexSolution* _S = new CplexSolution(_LMILP, [&](IloNumVarArray& X, IloNumArray& V) { context.getCandidatePoint(X, V); });
		//cout << "LazyCallBack: Integer Solution Obtained...: "<< getObjValue() << endl;
		int _OF = _S->getOF();

		double target = context.getIncumbentObjective(); vector< vector<int> > Cycles(_I->nvehicles());

		//We get the Routes
		vector< vector<int> > Routes(_I->nvehicles());
		for (int k = 0; k < _I->nvehicles(); ++k) {
			Routes[k] = _S->getRandomPath(k, _S->initialdepotid());
			assert(_S->isinitialdepot(Routes[k].front()));
			assert(_S->isfinaldepot(Routes[k].back()));
		}
		
		vector< vector<int> > RepairedRoutes = Routes;
		_S->adjustRoutesLength(RepairedRoutes);

		//We check if it is an acceptable solution
		double obj = context.getCandidateObjective();
		double fitness = _S->fitness(RepairedRoutes);
		//double fitness = _S->fitness(Routes);
		if ((_S->getOF() == 1 && fitness >= obj - EPS) || (_S->getOF() == 2 && fitness <= obj + EPS)) {
			cout << "* Integer Solution Accepted..." << obj << " * " << endl;
            searchSolutionFromLP(context);
			//saveSolGraph(context);
			return;
		}
		else cout << "Integer Solution Rejected..." << obj << " (" << fitness << ")" << endl;

		searchSolutionFromLP(context);

		CutPool Cuts([&](IloExpr& E) {return context.getCandidateValue(E); });

		//Routes Cuts
		collectCutsOnRoutes(Cuts, Routes, _S, target);

		//Flow Cuts
		collectCutsOnFlow(Cuts, _S, target);

		if (!USE_GSEC && Cuts.empty()) getCycleCuts(Cuts, _S, target);

		updatelog(context, Cuts);

		IloRangeArray LazyCuts(context.getEnv());

		for (auto Cut : Cuts) {
			LazyCuts.add(*Cut);
		}

		
		assert(LazyCuts.getSize() > 0);
		context.rejectCandidate(LazyCuts);
		LazyCuts.end();

		Cuts.clean();

		delete _S;
	}
	catch (IloException& ex) {
		cout << "Error Lazy: " << ex << endl;
	}

}

void MyCallback::addUserCuts(const IloCplex::Callback::Context& context)
{
	
	static double _BOUND = DBL_MAX;
	static int _ITER = 0;
	if (!context.inRelaxation()) return ;
	CPXLONG THREADID = context.getLongInfo(IloCplex::Callback::Context::Info::ThreadId);
	CPXLONG Node_LEFT = context.getLongInfo(IloCplex::Callback::Context::Info::NodesLeft);
	CPXINT AFTERCUTLOOP = context.getIntInfo(IloCplex::Callback::Context::Info::AfterCutLoop);
	double BestBound = context.getRelaxationObjective();// context.getDoubleInfo(IloCplex::Callback::Context::Info::BestBound);


	if (Node_LEFT>1 ) {
		if (THREADID == 0) {
			_BOUND = DBL_MAX;
			_ITER = 0;
		}
		return;
	}	
	if (!AFTERCUTLOOP) return;
	
	//if (Node_LEFT > 1 /*&& RAND01() > 0.01*/) saveGraph(context);
	//if (Node_LEFT > 1 && RAND01() > 0.01) return ;
	if (/*AFTERCUTLOOP &&*/ Node_LEFT == 1 && THREADID == 0) {
		if (_BOUND<DBL_MAX && fabs(BestBound - _BOUND)/ BestBound < 0.001)
			_ITER++;
		else {
			_BOUND = BestBound;
			_ITER = 0;
		}
		if (_ITER > 10) {
			cout << "Max Iteration Limit Reached..." << endl;
			//saveGraph(context);
			return ;
		}
	}
	
	try {

		searchSolutionFromLP(context);

		CplexSolution* _S = new CplexSolution(_LMILP, [&context](IloNumVarArray& X, IloNumArray& V) { context.getRelaxationPoint(X, V); });

		double target = context.getIncumbentObjective();

		CutPool Cuts([&](IloExpr& E) {return context.getRelaxationValue(E); });

		vector < pair<string, int> > Log;

		size_t MinCuts = SIZE_MAX;

		//if (_LMILP->_OF == 1 && BestBound - floor(BestBound) > EPS) {
		//	IloRange R(_LMILP->_env, -IloInfinity, _LMILP->_Sm, floor(BestBound), LazyMILP::ScoreCut.c_str());
		//	context.addUserCut(R, IloCplex::UseCutFilter, true);
		//	R.end();
		//}

		//if (Cuts.size() < MinCuts || _ITER > 1)
		collectLogicalConstraints(Cuts, _S, target);
		//if (Cuts.empty()) {
		//	collectTreeCuts(Cuts, _S, target);
		//}

		//Matching Cuts
		if (Cuts.size() < MinCuts) {
			//auto n = Cuts.size();
			//collectMatchingCutsPerVehicles(Cuts, _S);
			//if (Cuts.size() == n) 
			collectMatchingCutsV2(Cuts, _S, target);
			
		}

		//Flow Cuts
		if (Cuts.size() < MinCuts) {
			//auto n = Cuts.size();
			collectCutsOnFlowPerVehicles(Cuts, _S, target);
			//if (Cuts.size() == n) {
			//	collectExtraCutsOnFlowPerVehicles(Cuts, _S, target);
			//	if (n != Cuts.size()) cout << "WoW" << endl;
			//}
		}

		//Conflicts Cuts
		if (Cuts.size() < MinCuts) {
			//collectCutsOnConflicts(Cuts, _S, target, EPS);
		}


		
				
		

		updatelog(context, Cuts);

#if SAVE_ROOT_GRAPH
		if(Cuts.empty()) saveGraph(context);
#endif

		for (auto R : Cuts) context.addUserCut(*R, IloCplex::UseCutForce, false);


		Cuts.clean();
		
		delete _S;
		
	}
	catch (IloException& ex) {
		cout << "Error User Cuts: " << ex << endl;
	}



	
}

void MyCallback::searchSolutionFromLP(const IloCplex::Callback::Context& context)
{
	if (!USE_HEURISTIC) return;

	CplexSolution* _S = NULL;
	if (context.inCandidate())
		_S = new CplexSolution(_LMILP, [&context](IloNumVarArray& X, IloNumArray& V) { context.getCandidatePoint(X, V); });
	else if (context.inRelaxation() || context.inBranching()) {
		//long Node_ID = context.getLongInfo(IloCplex::Callback::Context::Info::NodeUID);
		CPXLONG Node_LEFT = context.getLongInfo(IloCplex::Callback::Context::Info::NodesLeft);
		_S = new CplexSolution(_LMILP, [&context](IloNumVarArray& X, IloNumArray& V) { context.getRelaxationPoint(X, V); });
		
	}
	else return;
	
	vector<vector<int> > Routes = _S->getHeuristicRoutes();


	_S->removeDuplicateNodes(Routes);
	_S->adjustRoutesLength(Routes);
	_S->optimizeRoutesLength(Routes);

	_S->improveRoutes(Routes, context.getIncumbentObjective());
	
	//_S->reorderRoutes(Routes); //For simmetry breaking constraint

	_S->reset();
	for (int k = 0; k < _I->nvehicles(); ++k) {
		int i = Routes[k].front();
		for (auto j : Routes[k]) if (j != i) {
			_S->setXVal(i, j, k, 1.0);
			i = j;
		}
	}

	if (context.inCandidate()) cout << "Heuristic Solution from Candidate: " << _S->fitness() << endl;
	//assert(_S->fitness() == _S->fitness(Routes));
	addSolution(context, _S);
	

	delete _S;

}

bool MyCallback::addSolution(const IloCplex::Callback::Context& context, CplexSolution* S)
{
	int _OF = S->getOF();
	double target = (_OF == 1) ? 0.0 : DBL_MAX;
	try {
		target = context.getIncumbentObjective();
	}
	catch (IloException& ex) {
		cout << "Error Adding Solution: " << ex << endl;
	}

	double fitness = S->fitness();

	bool add = (_OF == 1 && fitness > target + EPS) || (_OF == 2 && fitness < target - EPS);
	//add = add && (!context.inCandidate() || fabs(fitness - context.getCandidateObjective()) < EPS);
	if (!add) return add;

	cout << "Passing New solution from Heuristic: " << fitness << " (";
	if (context.inRelaxation()) cout << context.getRelaxationObjective() << ") LR" << endl;
	else if (context.inCandidate()) cout << context.getCandidateObjective() << ") C" << endl;
	else if (context.inBranching()) cout << context.getRelaxationObjective() << ") B" << endl;

	try {

		auto P = S->extractSolution();

		context.postHeuristicSolution(*(P.first), *(P.second), fitness, IloCplex::Callback::Context::SolutionStrategy::CheckFeasible);
		

		P.first->end(); delete P.first;
		P.second->end(); delete P.second;
	}
	catch (IloException& e) {
		cout << "Error: " << e << endl;
	}


	//Solution* Sol = new Solution(_I);
	//for (int k = 0; k < _I->nvehicles(); k++) {
	//	int i = 0;
	//	int j = 1;
	//	while (j != _I->nnodes()) {
	//		if (S->getXVal(i,j,k) < 0.5)
	//			j++;
	//		else if (S->getXVal(i, j, k) > 0.5) {
	//			if (i == 0) Sol->addNode(k, i);
	//			Sol->addNode(k, j);
	//			if (j == _I->nnodes() - 1) break;
	//			i = j;
	//			j = 1;
	//		}
	//	}
	//}
	//assert(Sol->isFeasible());
	//delete Sol;
	

	return add;
}


void MyCallback::collectLogicalConstraints(CutPool& Cuts, CplexSolution* S, double bestbound)
{
	//Logical Constraints
	//for (int i = 0; i < _I->nnodes(); ++i) if (!S->isdepot(i))
	//	for (int j = 0; j < i; ++j) {
	//		bool arecomp = false;
	//		for (int k = 0; k < _I->nvehicles() && !arecomp; ++k) arecomp = (_I->comp(k, i) && _I->comp(k, j));
	//		if (arecomp) Cuts.push(S->genLogicalConstraint(i, j));
	//	}

#if !PRELOAD_LC
	for (int k = 0; k < _I->nvehicles(); ++k)
		for (int i : _I->reachablenodes()) if (_I->comp(k, i))
			for (int j = 0; j < i; ++j) if (_I->comp(k, j)) {
				if (!S->isdepot(i)) Cuts.push(S->genLogicalConstraint(i, j, k));
				if (!S->isdepot(j)) Cuts.push(S->genLogicalConstraint(j, i, k));
			}
#endif



	for (int k = 0; k < _I->nvehicles(); ++k) {
		IloNumArray Y = S->getYVal(k);
		vector<int> N;
		for (int i = 0; i < _I->nnodes(); ++i) if (Y[i] > EPS) N.push_back(i);
		sort(N.begin(), N.end(), [Y](int i, int j) {return Y[i] > Y[j]; });
		auto it = N.begin();
		int n = 0;
		double sumy = 0.0;
		while (it != N.end() && sumy + Y[*it] > n) {
			sumy += Y[*it];
			n++;
			it++;
		}
		Y.end();
		N.erase(it, N.end());
		double dmin = minSpanningTreeLength(N);
		if (dmin > _I->tmax()) {
			IloExpr expr(_LMILP->_env);
			for (auto i : N) expr += _LMILP->_y[i][k];
			IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, IloNum(N.size() - 1), LazyMILP::TmaxCut.c_str());
			expr.end();
			bool added = Cuts.push(R);
			//assert(added);
			//if (added) cout << *R << endl;
		}
		//else if (S->score(N) <= bestbound) {
		//	bool added = Cuts.push(S->genPathCoverCut(N, k));
		//	//if (added) cout << "Cycle Cover Cut added" << endl;
		//}

	}


	////Cover Inequalities
	//for (int k = 0; k < _I->nvehicles(); ++k) {
	//	IloNumArray Y = S->getYVal(k);
	//	vector<edge> E;
	//	for (int i = 0; i < _I->nnodes(); ++i) for (int j = 0; j < i; j++) {
	//		double w = (1 - S->getXVal(i, j, k));
	//		double d = _I->distance(_I->node(i), _I->node(j));
	//		//assert(w <= 1.0 + EPS);
	//		if (w < 1.0 - EPS) E.push_back(edge(i, j, w / d));
	//	}
	//	Y.end();
	//	sort(E.begin(), E.end(), [](edge& a, edge& b) {return a.w < b.w; });
	//	double d = 0;
	//	double z = 0;
	//	vector<edge> T;
	//	for (auto e : E) {
	//		d += _I->distance(_I->node(e.i), _I->node(e.j));
	//		z += (1 - S->getXVal(e.i, e.j, k));
	//		T.push_back(e);
	//		if (d > _I->tmax()) break;
	//	}

	//	if (z >= 1 - EPS) continue;
	//	//vector<int> V;
	//	//for (auto e : T) {
	//	//	assert(_I->comp(k, e.i) && _I->comp(k, e.j));
	//	//	V.push_back(e.i);
	//	//	V.push_back(e.j);
	//	//}
	//	//sort(V.begin(), V.end());
	//	//auto last=unique(V.begin(), V.end());
	//	//V.erase(last, V.end());

	//	//bool added=Cuts.push(S->genPathCut(V, k));
	//	//if (added) cout << *Cuts.back() << endl;

	//	IloExpr expr(_LMILP->_env);
	//	for (auto& e : T) expr += _LMILP->_x[_LMILP->getarcid(e.i, e.j, k)];

	//	IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, (int)T.size() - 1, LazyMILP::EdgeCut.c_str());

	//	bool added = Cuts.push(R);
	//	if (added) cout << *Cuts.back() << endl;
	//}

	//The conflicts
	//vector<int> Y(_I->nnodes(), 0.0);
	//for (int k = 0; k < _I->nvehicles(); ++k) {
	//	IloNumArray YVal = S->getYVal(k);
	//	for (int i = 0; i < _I->nnodes(); i++) Y[i] += YVal[i];
	//	YVal.end();
	//}
	//vector<int> nodes;
	//for (int i = 0; i < _I->nnodes(); i++) if (Y[i] > EPS) nodes.push_back(i);
	//for (auto& C : _ConflictPairs) if (Y[C.first] > EPS && Y[C.second] > EPS) {
	//	int nodefirst = *max_element(nodes.begin(), nodes.end(), [&](int i, int j) {return S->getXVal(C.first, i, k) < S->getXVal(C.first, j, k); });
	//	int nodesecond = *max_element(nodes.begin(), nodes.end(), [&](int i, int j) {return S->getXVal(C.second, i, k) < S->getXVal(C.second, j, k); });

	//	int edgefirst = _LMILP->getarcid(C.first, nodefirst, k);
	//	int edgesecond = _LMILP->getarcid(C.second, nodefirst, k);

	//	assert(edgefirst != edgesecond);

	//	IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, _LMILP->_x[edgefirst] + _LMILP->_x[edgesecond] - _LMILP->_y[C.first][k] - _LMILP->_y[C.second][k], 0, "LogCompCut");

	//	Cuts.push(R);

	//}
	
	
	
}

void MyCallback::collectCutsOnRoutes(CutPool& Cuts, const vector<vector<int>>& Routes, CplexSolution* S, double target)
{
	for (auto& p : Routes) {

		assert (S->isinitialdepot(p.front()));
		assert (S->isfinaldepot(p.back()));

		vector<int> corepath(p);
		corepath.erase(corepath.begin());
		corepath.pop_back();

		double xval = S->getXVal(p);
		double xvalcore = S->getXVal(corepath);
		double score = S->score(p);

		bool cutfound = false;

		//No vehicle can visit this set if V is empty
		auto V = S->SetVehicleCompatible(p);
		if (!cutfound && V.empty()) {
			cutfound = Cuts.push(S->genPathCut(corepath));
		}
		
		//Path Cut (based on score)
		if (!cutfound && S->getOF() == 1 && score <= target ) {
			cutfound = Cuts.push(S->genPathCoverCut(p));
		}
		
		if (!cutfound && S->getOF() == 2 && fabs(score - S->getmu()) >= target ) {
				//Path Cut (based on score)
				IloRange* Cut = NULL;
				if (score > S->getmu()) {
					cutfound = Cuts.push(S->genPathCut(corepath));
				}
				else cutfound = Cuts.push(S->genPathCoverCut(p));
			}
			
		if (!cutfound && S->length(p) > _I->tmax()) {
			//Cut based on length
			cutfound = Cuts.push(S->genInfPathCut(p));
			if (p.size() > 3) {
				vector<int> rPath(p);
				reverse(rPath.begin() + 1, rPath.end() - 2);
				if (S->length(rPath) > _I->tmax())
					cutfound = Cuts.push(S->genInfPathCut(rPath));
			}
		}
		
		

		
		

	}

	////Logical Constraints
	//for (int i = 0; i < _I->nnodes(); ++i) if (!S->isdepot(i))
	//	for (int j = 0; j < _I->nnodes(); ++j) if (i != j) {
	//		Cuts.push(S->genLogicalConstraint(i, j));
	//	}

}

void MyCallback::collectCutsOnFlow(CutPool& Cuts, CplexSolution* S, double target, double threshold)
{
	vector<double> isCandidate(_I->nnodes(), true);
	isCandidate[S->initialdepotid()] = false;
	isCandidate[S->finaldepotid()] = false;
	vector<double> Y(_I->nnodes(), 0.0);
	for (int k = 0; k < _I->nvehicles(); ++k) {
		IloNumArray YVal = S->getYVal(k);
		for (int i = 0; i < _I->nnodes(); ++i)
			Y[i] += YVal[i];
		YVal.end();
	}

	vector<int> nodes;
	for (int i = 0; i < _I->nnodes(); ++i) if (Y[i] > threshold && !S->isdepot(i)) nodes.push_back(i);
	sort(nodes.begin(), nodes.end(), [Y](int i, int j) {return Y[i] > Y[j]; });

	for(auto i : nodes) if (isCandidate[i]){
		vector<int> Sources;
		Sources.push_back(i);

		vector<int> Targets;
		Targets.push_back(S->initialdepotid());
		Targets.push_back(S->finaldepotid());

		auto P = S->getShoresMinCut(Sources, Targets);
		assert(find(P.first.begin(), P.first.end(), S->initialdepotid()) == P.first.end());
		assert(find(P.first.begin(), P.first.end(), S->finaldepotid()) == P.first.end());

		double flow(S->getFlowValue(P.first, P.second));
		//assert(S->getFlowValue(P.first, P.second) <= 2+EPS);
		//if (flow > EPS) continue;
		//assert(flow < 2.1);
		if (flow >= 2 - EPS) continue;

		bool cutfound = false;

		//auto K = S->SetVehicleCompatible(P.first);
		//if (!cutfound && K.empty()) {
		//	cutfound = Cuts.push(S->genPathCut(P.first));
		//}

		////Infeasible path
		//if (!cutfound && S->getOF()==2 && P.first.size() >= 2 && S->score(P.first) >= S->getmu() + target) {
		//	cutfound = Cuts.push(S->genPathCut(P.first));
		//}

		//Connectivity cuts
		if (!cutfound) {
			//int u = *max_element(P.first.begin(), P.first.end(), [Y](int i, int j) {return Y[i] < Y[j]; });
			//cutfound = Cuts.push(S->genFlowCut(P.first, P.second, u));
			//for(int u : P.first) cutfound = Cuts.push(S->genFlowCut(P.first, P.second, u)) || cutfound;;
			cutfound = Cuts.push(S->genFlowCut(P.first, P.second, i));
		}

		//Subtour elimination
		if (!cutfound && P.first.size() > 1) {
			cutfound = Cuts.push(S->genCycleCut(P.first));
		}

		


		if(cutfound) for (auto s : P.first) isCandidate[s] = false;
		
	}

	for (auto i : nodes) isCandidate[i] = true;
	for (auto i : nodes) if (isCandidate[i]) {
		vector<int> Sources;
		Sources.push_back(S->initialdepotid());
		Sources.push_back(S->finaldepotid());
		vector<int> Targets;
		Targets.push_back(i);
		auto P = S->getShoresMinCut(Sources, Targets);
		assert(find(P.first.begin(), P.first.end(), S->initialdepotid()) != P.first.end());
		assert(find(P.first.begin(), P.first.end(), S->finaldepotid()) != P.first.end());


		bool cutfound = false;

		//auto K = S->SetVehicleCompatible(P.first);
		//if (!cutfound && K.empty()) {
		//	cutfound = Cuts.push(S->genPathCut(P.first));
		//	//if(!cutfound) cutfound = Cuts.push(S->genCycleCoverCut(P.first));
		//}

		//Infeasible path
		//if (!cutfound && S->getOF() == 1 && P.first.size() >= 2 && S->score(P.first) <= target) {
		//	cutfound = Cuts.push(S->genPathCut(P.first));
		//	//if (!cutfound) cutfound = Cuts.push(S->genCycleCoverCut(P.first));
		//}

		double flow(S->getFlowValue(P.first, P.second));
		if (flow >= 2 - EPS) continue;

		//Subtour elimination
		if (!cutfound ) {
			//cutfound = Cuts.push(S->genCycleCut(P.first));
			cutfound = Cuts.push(S->genCycleCoverCut(P.first));
		}

		//Connectivity cuts
		if (!cutfound) {
			//int u = *max_element(P.first.begin(), P.first.end(), [Y](int i, int j) {return Y[i] < Y[j]; });
			//cutfound = Cuts.push(S->genFlowCut(P.first, P.second, u));
			//for(int u : P.first) cutfound = Cuts.push(S->genFlowCut(P.first, P.second, u)) || cutfound;;
			cutfound = Cuts.push(S->genSECCut(P.first, P.second, i));
		}



		if (cutfound) for (auto s : P.second) isCandidate[s] = false;


	}

}

void MyCallback::collectCutsOnFlowPerVehicles(CutPool& Cuts, CplexSolution* S, double target)
{
	
	for (int k = 0; k < _I->nvehicles(); ++k) {

		vector<double> isCandidate(_I->nnodes(), false);
		IloNumArray Y = S->getYVal(k);
		
		vector<int> nodes;
		for (int i = 0; i < _I->nnodes(); ++i) if (Y[i] > EPS && !S->isdepot(i)) {
			nodes.push_back(i);
			isCandidate[i] = true;
		}
		sort(nodes.begin(), nodes.end(), [Y](int i, int j) {return Y[i] > Y[j]; });
		
		for (auto i : nodes) if (isCandidate[i]) {
			vector<int> sources;
			sources.push_back(S->initialdepotid());
			sources.push_back(S->finaldepotid());
			vector<int> targets;
			targets.push_back(i);
			auto Pk = S->getShoresMinCut(sources, targets, k);
			assert(_I->comp(k, i));
			bool cutfound = false;

			assert(find(Pk.first.begin(), Pk.first.end(), S->initialdepotid()) != Pk.first.end());
			assert(find(Pk.first.begin(), Pk.first.end(), S->finaldepotid()) != Pk.first.end());



			//Conditional cut
			double score = S->score(Pk.first);
			if(!cutfound && S->getOF()==1 && score<=target)
				cutfound =Cuts.push(S->genPathCoverCut(Pk.first, k));
			
	
			//Conflict Cuts
			//if (!cutfound && containsConflicts(Pk.first))
			//	cutfound = Cuts.push(S->genPathCut(Pk.first, k));
			
			double flow(S->getFlowValue(Pk.first, Pk.second, k));
			//if (flow >= 2 - EPS) continue;

			//Subtour Elimination
			if (!cutfound && Pk.first.size() > 1)
				cutfound = Cuts.push(S->genCycleCoverCut(Pk.first, k));

			//SEC
			if (!cutfound)
				cutfound = Cuts.push(S->genSECCut(Pk.first, Pk.second, i, k)) || cutfound;
			

			if (cutfound) for (auto s : Pk.second) isCandidate[s] = false;

			
		}

		for (int i : nodes) isCandidate[i] = true;

		for (auto i : nodes) if (isCandidate[i]) {
			vector<int> sources;
			sources.push_back(i);
			
			vector<int> targets;
			targets.push_back(S->initialdepotid());
			targets.push_back(S->finaldepotid());
			
			auto Pk = S->getShoresMinCut(sources, targets, k);
			assert(_I->comp(k, i));
			bool cutfound = false;

			assert(find(Pk.first.begin(), Pk.first.end(), S->initialdepotid()) == Pk.first.end());
			assert(find(Pk.first.begin(), Pk.first.end(), S->finaldepotid()) == Pk.first.end());

			

			//Conditional cut
			if (!cutfound && S->getOF() == 2 && Pk.first.size() >= 2 && S->score(Pk.first) >= S->getmu() + target) 
				cutfound = Cuts.push(S->genPathCut(Pk.first, k));
			
			
			//Conflict Cuts
			//if (!cutfound && containsConflicts(Pk.first))
			//	cutfound = Cuts.push(S->genPathCut(Pk.first, k));
			
			double flow(S->getFlowValue(Pk.first, Pk.second, k));
			//if (flow >= 2 - EPS) continue;

			//Subtour Elimination
			if (!cutfound && Pk.first.size() > 1) {
				cutfound = Cuts.push(S->genCycleCut(Pk.first, k));
				//cutfound = Cuts.push(S->genCycleCut(Pk.first));
				//if(!cutfound)
				//	cutfound = Cuts.push(S->genCycleCut(Pk.first, k));
			}

			//Connectivity
			if (!cutfound )
				cutfound = Cuts.push(S->genFlowCut(Pk.first, Pk.second, i, k)) || cutfound;

			//Conditional cut
			double score = S->score(Pk.first);
			if (!cutfound && S->getOF() == 1 && score <= target)
			{
				vector<int> H(Pk.first);
				H.push_back(S->initialdepotid());
				H.push_back(S->finaldepotid());
				cutfound = Cuts.push(S->genPathCoverCut(H, k));
			}

			if (cutfound) for (auto s : Pk.first) isCandidate[s] = false;

		}

		Y.end();
	}
}

void MyCallback::collectExtraCutsOnFlowPerVehicles(CutPool& Cuts, CplexSolution* S, double target)
{

	for (int k = 0; k < _I->nvehicles(); ++k) {

		vector<double> isCandidate(_I->nnodes(), false);
		IloNumArray Y = S->getYVal(k);

		vector<int> nodes;
		for (int i = 0; i < _I->nnodes(); ++i) if (Y[i] > EPS && !S->isdepot(i)) {
			nodes.push_back(i);
			isCandidate[i] = true;
		}
		sort(nodes.begin(), nodes.end(), [Y](int i, int j) {return Y[i] > Y[j]; });


		for (auto i : nodes) if (isCandidate[i]) {
			vector<int> sources;
			sources.push_back(S->initialdepotid());
			sources.push_back(i);
			vector<int> targets;
			targets.push_back(S->finaldepotid());
			auto Pk = S->getShoresMinCut(sources, targets, k);
			assert(_I->comp(k, i));
			bool cutfound = false;

			assert(find(Pk.first.begin(), Pk.first.end(), S->initialdepotid()) != Pk.first.end());


			//Conditional cut not working
			double score = S->score(Pk.first);
			if (!cutfound && S->getOF() == 1 && score <= target) {
				vector<int> H(Pk.first);
				H.push_back(S->finaldepotid());
				cutfound = Cuts.push(S->genPathCoverCut(H, k));
			}

			if (!cutfound && containsConflicts(Pk.first))
				cutfound = Cuts.push(S->genPathCut(Pk.first, k));

			double flow(S->getFlowValue(Pk.first, Pk.second, k));
			//if (flow >= 1 - EPS) continue;

			//Subtour Elimination
			if (!cutfound && Pk.first.size() > 1) {
				cutfound = Cuts.push(S->genCycleCut(Pk.first, k));
			}

			//Flow Cut
			if (!cutfound /*&& Pk.first.size() > 3*/)
				cutfound = Cuts.push(S->genFlowCut(Pk.first, Pk.second, i, k));

			////if (!cutfound) {
			//	int t = -1;
			//	for (int l : Pk.second) if (!S->isdepot(l) && (t < 0 || Y[t] < Y[l])) t = l;
			//	if (t >= 0) {
			//		assert(!S->isdepot(t));
			//		/*cutfound = */Cuts.push(S->genConnectFlowCut(Pk.first, Pk.second, i, t, k));
			//	}
			////}


			if (cutfound) for (auto s : Pk.second) isCandidate[s] = false;

		}

		for (int i : nodes) isCandidate[i] = true;

		for (auto i : nodes) if (isCandidate[i]) {
			vector<int> sources;
			sources.push_back(S->initialdepotid());
			vector<int> targets;
			targets.push_back(S->finaldepotid());
			targets.push_back(i);
			auto Pk = S->getShoresMinCut(sources, targets, k);
			assert(_I->comp(k, i));
			bool cutfound = false;

			assert(find(Pk.first.begin(), Pk.first.end(), S->initialdepotid()) != Pk.first.end());


			//Conditional cut not working
			double score = S->score(Pk.first);
			if (!cutfound && S->getOF() == 1 && score <= target) {
				vector<int> H(Pk.first);
				H.push_back(S->finaldepotid());
				cutfound = Cuts.push(S->genPathCoverCut(H, k));
			}
				
			//Conflict Cuts
			if (!cutfound && containsConflicts(Pk.first))
				cutfound = Cuts.push(S->genPathCut(Pk.first, k));

			double flow(S->getFlowValue(Pk.first, Pk.second, k));
			//if (flow >= 1 - EPS) continue;

			//Subtour Elimination
			if (!cutfound && Pk.first.size() > 1) {
				cutfound = Cuts.push(S->genCycleCut(Pk.first, k));
			}

			//SEC
			if (!cutfound && Pk.first.size() > 3)
				cutfound = Cuts.push(S->genSECCut(Pk.first, Pk.second, i, k));

			////if (!cutfound) {
			//	int s = -1;
			//	for (int l : Pk.first) if (!S->isdepot(l) && (s < 0 || Y[s] < Y[l])) s = l;
			//	if (s >= 0) {
			//		assert(!S->isdepot(s));
			//		/*cutfound = */Cuts.push(S->genConnectFlowCut(Pk.first, Pk.second, s, i, k));
			//	}
			////}

			if (cutfound) for (auto s : Pk.second) isCandidate[s] = false;

		}

		//for (auto i : nodes) for (auto j : nodes) if(i!=j){
		//	vector<int> sources;
		//	sources.push_back(i);
		//	vector<int> targets;
		//	targets.push_back(j);
		//	auto Pk = S->getShoresMinCut(sources, targets, k);

		//	bool cutfound = false;

		//	double flow(S->getFlowValue(Pk.first, Pk.second, k));
		//	//if (flow >= 2 - EPS) continue;

		//	////Subtour Elimination
		//	//if (!cutfound && Pk.first.size() > 1) {
		//	//	cutfound = Cuts.push(S->genCycleCut(Pk.first, k));
		//	//}

		//	////SEC
		//	//if (!cutfound && Pk.first.size() > 3)
		//	//	cutfound = Cuts.push(S->genSECCut(Pk.first, Pk.second, i, k));

		//	if (!cutfound) 
		//		cutfound = Cuts.push(S->genConnectFlowCut(Pk.first, Pk.second, i, j, k));
		//		
		//	

		//	//if (cutfound) for (auto s : Pk.second) isCandidate[s] = false;

		//}

		Y.end();
	}
}

void MyCallback::collectCutsOnMultiFlow(CutPool& Cuts, CplexSolution* S, double threshold)
{
	vector< vector<int> > Candidates(_I->nvehicles());
	//threshold = fmax(1.0 - 1 / (double)_I->nvehicles(), threshold);
	for (int k = 0; k < _I->nvehicles(); ++k) {
		IloNumArray YVal = S->getYVal(k);
		for (int i = 0; i < _I->nnodes(); ++i) if (YVal[i] > threshold && YVal[i] < 1.0 - EPS && !S->isdepot(i))
			Candidates[k].push_back(i);
		YVal.end();
	}

	for (int k = 0; k < _I->nvehicles(); ++k) {
		vector<bool> isvisited(_I->nnodes(), false);
		for (int i  : Candidates[k]) isvisited[i]= true;
		isvisited[S->initialdepotid()] = isvisited[S->finaldepotid()] = true;

		for (int i : Candidates[k]) {
			vector<int> Ki;
			vector<int> Kj;
			for (int l = 0; l < _I->nvehicles(); l++)
				if (find(Candidates[l].begin(), Candidates[l].end(), i) != Candidates[l].end()) Ki.push_back(l);
				else Kj.push_back(l);

			for (int l : Kj) if (l < k) {
				for (int j : Candidates[l]) if (!isvisited[l] /*&& _I->comp(k,j)*/) {
					assert(i != j);

					vector<int> Sources;
					Sources.push_back(i);
					Sources.push_back(j);

					vector<int> Targets;
					Targets.push_back(S->initialdepotid());
					Targets.push_back(S->finaldepotid());

					auto Shores = S->getShoresMinCut(Sources, Targets);

					assert(find(Shores.first.begin(), Shores.first.end(), S->initialdepotid()) == Shores.first.end());
					assert(find(Shores.first.begin(), Shores.first.end(), S->finaldepotid()) == Shores.first.end());

					double flow = S->getFlowValue(Shores.first, Shores.second);

					if (flow >= 4.0 - EPS)  continue;

					vector< vector<int> > K;
					K.push_back(Ki);
					K.push_back(Kj);
					bool added = Cuts.push(S->genMultiFlowCut(Shores.first, Shores.second, Sources, K));
					if (added) cout << "MultiFlow added" << endl;

				}
			}
			


		}
	}

	

	//We do an equipartitions of vehicles
	//int n = 2;
	//vector< vector<int> > K(n);
	//vector< vector<int> > N(n);
	//for (int k = 0; k < _I->nvehicles(); ++k) {
	//	int pbest = 0;
	//	for (int p = 0; p < n; ++p) if (N[p].size() < N[pbest].size()) pbest = p;
	//	K[pbest].push_back(k);
	//	N[pbest].insert(N[pbest].end(), Candidates[k].begin(), Candidates[k].end());
	//}

	////int sum = accumulate(N.begin(), N.end(), 0.0, [](int a, vector<int> s) {return a + s.size(); });
	//

	//for (auto i : N[0]) for (auto j : N[1]) {
	//	bool arecomp = false;
	//	for (int k = 0; k < _I->nvehicles() && !arecomp; ++k)
	//		arecomp = (_I->comp(k, i) && _I->comp(k, j));

	//	if (!arecomp) continue;

	//	assert(i != j);
	//	vector<int> Sources;
	//	Sources.push_back(i);
	//	Sources.push_back(j);

	//	vector<int> Targets;
	//	Targets.push_back(S->initialdepotid());
	//	Targets.push_back(S->finaldepotid());

	//	auto Shores = S->getShoresMinCut(Sources, Targets);

	//	assert(find(Shores.first.begin(), Shores.first.end(), S->initialdepotid()) == Shores.first.end());
	//	assert(find(Shores.first.begin(), Shores.first.end(), S->finaldepotid()) == Shores.first.end());

	//	double flow = S->getFlowValue(Shores.first, Shores.second);

	//	if (flow >= 4.0 - EPS)  continue;

	//	bool added=Cuts.push(S->genMultiFlowCut(Shores.first, Shores.second, Sources, K));
	//	//if (added) break;
	//	//	cout << "MultiFlow Cut found" << endl;
	//}
}

void MyCallback::collectCutsOnConflicts(CutPool& Cuts, CplexSolution* S, double target, double threshold)
{

	vector<double> Y(_I->nnodes(), 0.0);
	for (int k = 0; k < _I->nvehicles(); ++k) {
		IloNumArray YVal = S->getYVal(k);
		for (int i = 0; i < _I->nnodes(); ++i)
			Y[i] += YVal[i];
		YVal.end();
	}

	vector< pair<int, int> > conflict;
	//vector< pair<int, int> > _ConflictPairs = _I->conflictnodepairs();
	copy_if(_ConflictPairs.begin(), _ConflictPairs.end(), back_inserter(conflict), [Y,threshold](pair<int, int> a) {return Y[a.first] > threshold && Y[a.second] > threshold; });

	sort(conflict.begin(), conflict.end(), [Y](pair<int, int> a, pair<int, int> b) {return Y[a.first] + Y[a.second] < Y[b.first] + Y[b.second]; });
	//assert(!hasDuplicates(conflict));
	while (!conflict.empty()) {
		auto& C = conflict.back();
		conflict.pop_back();

		vector<int> Sources;
		Sources.push_back(C.first);
		Sources.push_back(C.second);

		vector<int> Targets;
		Targets.push_back(S->initialdepotid());
		Targets.push_back(S->finaldepotid());

		auto Shores = S->getShoresMinCut(Sources, Targets);

		//assert(find(Shores.first.begin(), Shores.first.end(), S->initialdepotid()) == Shores.first.end());
		//assert(find(Shores.first.begin(), Shores.first.end(), S->finaldepotid()) == Shores.first.end());

		double flow = S->getFlowValue(Shores.first, Shores.second);

		if (flow >= 4.0 - EPS)  continue;

		bool cutfound = false;


		//if (!cutfound /*&& Shores.first.size() > 2*/) 
		//	cutfound = Cuts.push(S->genPathCut(Shores.first));


		if(!cutfound)
			cutfound = Cuts.push(S->genIncompatibilityCut(Shores.first, Shores.second, C.first, C.second)) || cutfound;

		//assert(!cutfound);

		if (cutfound) {
			auto it = conflict.begin();
			while (it != conflict.end()) {
				auto iti = find(Shores.first.begin(), Shores.first.end(),it->first);
				auto itj = find(Shores.first.begin(), Shores.first.end(), it->second);
				if (iti != Shores.first.end() && itj!=Shores.first.end()) it = conflict.erase(it);
				else it++;
			}
		}

		//if (cutfound) break;
	}

	//copy_if(_ConflictPairs.begin(), _ConflictPairs.end(), back_inserter(conflict), [Y, threshold](pair<int, int> a) {return Y[a.first] > threshold && Y[a.second] > threshold; });
	//sort(conflict.begin(), conflict.end(), [Y](pair<int, int> a, pair<int, int> b) {return Y[a.first] + Y[a.second] < Y[b.first] + Y[b.second]; });
	////assert(!hasDuplicates(conflict));
	//while (!conflict.empty()) {
	//	auto& C = conflict.back();
	//	conflict.pop_back();

	//	vector<int> Sources;
	//	Sources.push_back(S->initialdepotid());
	//	Sources.push_back(S->finaldepotid()); 

	//	vector<int> Targets;
	//	Targets.push_back(C.first);
	//	Targets.push_back(C.second);

	//	auto Shores = S->getShoresMinCut(Sources, Targets);

	//	assert(find(Shores.first.begin(), Shores.first.end(), S->initialdepotid()) != Shores.first.end());
	//	assert(find(Shores.first.begin(), Shores.first.end(), S->finaldepotid()) != Shores.first.end());

	//	double flow = S->getFlowValue(Shores.first, Shores.second);

	//	if (flow >= 4.0 - EPS)  continue;

	//	bool cutfound = false;

	//	//double score = S->score(Shores.first);
	//	//if (score < target)
	//	//	cutfound = Cuts.push(S->genPathCoverCut(Shores.first));

	//	if (!cutfound) {
	//		cutfound = Cuts.push(S->genIncompatibilityCut(Shores.first,Shores.second, C.first,C.second));
	//	}


	//	if (cutfound) {
	//		auto it = conflict.begin();
	//		while (it != conflict.end()) {
	//			auto iti = find(Shores.first.begin(), Shores.first.end(), it->first);
	//			auto itj = find(Shores.first.begin(), Shores.first.end(), it->second);
	//			if (iti != Shores.first.end() && itj != Shores.first.end()) it = conflict.erase(it);
	//			else it++;
	//		}
	//	}

	//	//if (cutfound) break;
	//}

}

void MyCallback::collectCutsOnConflictsPath(CutPool& Cuts, CplexSolution* S, double target, double threshold)
{
	vector<double> Y(_I->nnodes(), 0.0);
	for (int k = 0; k < _I->nvehicles(); ++k) {
		IloNumArray YVal = S->getYVal(k);
		for (int i = 0; i < _I->nnodes(); ++i)
			Y[i] += YVal[i];
		YVal.end();
	}

	vector< pair<int, int> > conflict;
	copy_if(_ConflictPairs.begin(), _ConflictPairs.end(), back_inserter(conflict), [Y, threshold](pair<int, int> a) {return Y[a.first] > threshold && Y[a.second] > threshold; });

	sort(conflict.begin(), conflict.end(), [Y](pair<int, int> a, pair<int, int> b) {return Y[a.first] + Y[a.second] < Y[b.first] + Y[b.second]; });
	
	double** Network = S->extractCompleteNetwork();
	
	//assert(!hasDuplicates(conflict));
	while (!conflict.empty()) {
		auto& C = conflict.back();
		conflict.pop_back();

		auto Path = S->bfs(Network, _I->nnodes(), C.first,C.second);

		if (Path.empty()) continue;

		bool cutfound = false;

		if (!cutfound) {
			cutfound = Cuts.push(S->genPathCut(Path));
		}

		//if (!cutfound) {
		//	cutfound = Cuts.push(S->genIncompatibilityCut(Shores.first, Shores.second, C.first, C.second));
		//}

		if (cutfound) {
			auto it = conflict.begin();
			while (it != conflict.end()) {
				auto iti = find(Path.begin(), Path.end(), it->first);
				auto itj = find(Path.begin(), Path.end(), it->second);
				if (iti != Path.end() && itj != Path.end()) it = conflict.erase(it);
				else it++;
			}
		}

	}

}

void MyCallback::collectMatchingCuts(CutPool& Cuts, CplexSolution* S, double threshold)
{
	vector<int> K;
	for (int k = 0; k < _I->nvehicles(); ++k) K.push_back(k);

	vector<double> Y(_I->nnodes(), 0.0);
	for (int k = 0; k < _I->nvehicles(); ++k) {
		IloNumArray Yk = S->getYVal(k);
		for (int i = 0; i < _I->nnodes(); ++i)  Y[i] += Yk[i];	
		Yk.end();
	}

	vector<double> X;
	for (int i = 0; i < _I->nnodes(); ++i) if (Y[i] > EPS && !S->isdepot(i))
		X.push_back(ceil(Y[i] * 100) / 100.0);


	sort(X.begin(), X.end());
	auto last = unique(X.begin(), X.end());
	X.erase(last, X.end());

	sort(X.begin(), X.end(), [](double a, double b) {return a > b; });
	size_t n = Cuts.size();
	for (auto t : X) {
		collectMatchingCuts(Cuts, S, K, t);
		//if (n != Cuts.size()) break;
	}
	
}

void MyCallback::getCycleCuts(CutPool& Cuts, CplexSolution* S, double target)
{
	//For each vehicle we first collect all cycles if any
	for (int k = 0; k < _I->nvehicles(); ++k) {
		auto Y = S->getYVal(k);

		vector<int> visited(_I->nnodes(), false);

		vector< vector<int> > Cycles;
		//We collect all cycles
		for (int i = 0; i < _I->nnodes(); ++i) if (Y[i] > EPS && !visited[i] && !S->isdepot(i)) {
			vector<int> Cycle = S->getCycle(i);
			if (!Cycle.empty()) {
				Cycles.push_back(Cycle);
				for (int j : Cycle) visited[j] = true;
			}
		}

		//We add the cycle cuts
		for (auto& Cycle : Cycles) {
			//We Force a Cycle Cut
			IloExpr expr(_LMILP->_env);
			for (auto i : Cycle) for (auto j : Cycle) if (j < i)
				expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];

			IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, (int)Cycle.size() - 1, "ForcedCC");

			expr.end();
			assert(Cuts.push(R));
		}

		Y.end();
	}
	
}

void MyCallback::addBranches(const IloCplex::Callback::Context& context)
{
	try {
		if (!context.inBranching()) return;
		//IloNum BOUND = context.getDoubleInfo(IloCplex::Callback::Context::Info::BestBound);
		IloNum z = context.getRelaxationObjective();

		CplexSolution* _S = NULL;
		
		IloArray< IloNumArray > y(_LMILP->_env, _I->nvehicles());

		for (int k = 0; k < _I->nvehicles(); ++k) {
			y[k] = IloNumArray(_LMILP->_env);
			IloNumVarArray yvar(_LMILP->_env);
			for (int i : _I->reachablenodes()) yvar.add(_LMILP->_y[i][k]);
			context.getRelaxationPoint(yvar, y[k]);
			yvar.end();
		}

		bool branchcreated = false;
		auto& Reachable = _I->reachablenodes();

		if (!branchcreated) {
			int ibest = -1;
			int kbest = -1;
			double evbest = DBL_MAX;
			double scorebest = -1;
			
			for (int pos = 0; pos < Reachable.size();++pos )  {
				int i = Reachable[pos];
				if (_LMILP->isdepot(i)) continue;

				for (int k = 0; k < _I->nvehicles(); ++k){
					double val = y[k][pos];
					if (val > 1.0 - EPS) break;

					//double d = _I->distance(_I->node(_LMILP->initialdepotid()), _I->node(i)) + _I->distance(_I->node(_LMILP->finaldepotid()), _I->node(i));
					double ev = fabs(0.5 - val);
					if (ev >= 0.5 - EPS) continue;

					double score = fmin(1 - val, val) * _I->node(i)->score();
					if (score > scorebest + EPS || (fabs(score - scorebest) < EPS && evbest > ev)) {
						ibest = i;
						evbest = ev;
						kbest = k;
						scorebest = score;
					}
				}
			}
			if (evbest < 0.5 - EPS) {
				assert(ibest >= 0);

				context.makeBranch(_LMILP->_y[ibest][kbest], 1, IloCplex::BranchUp, z);
				context.makeBranch(_LMILP->_y[ibest][kbest], 0, IloCplex::BranchDown, z);
				branchcreated = true;
			}

		}


		if (!branchcreated) {

			


			if (_S == NULL)
				_S = new CplexSolution(_LMILP, [&context](IloNumVarArray& X, IloNumArray& V) { context.getRelaxationPoint(X, V); });

			int aidbest = -1;
			double evbest = DBL_MAX;
			double scorebest = 0;

			for (int i : Reachable) for (int j = 0; j < i; j++) {
				for (int k = 0; k < _I->nvehicles(); ++k) {
					int aid = _LMILP->getarcid(i, j, k);
					arc* A = _LMILP->_Arcs[aid];
					double val = _S->getXVal(aid);
					if (val > 1.0 - EPS) break;
					double ev = fabs(0.5 - val);
					double score = fmin(1.0 - val, val) * (_I->node(A->geti())->score() + _I->node(A->getj())->score());
					if (ev < 0.5 - EPS && (score > scorebest + EPS || (fabs(score - scorebest) < EPS && evbest > ev))) {
						aidbest = aid;
						evbest = ev;
						scorebest = score;
					}
				}
			}

			if (evbest < 0.5 - EPS) {
				assert(aidbest >= 0);

				context.makeBranch(_LMILP->_x[aidbest], 1, IloCplex::BranchUp, z);
				context.makeBranch(_LMILP->_x[aidbest], 0, IloCplex::BranchDown, z);
				branchcreated = true;
			}

		}
		 
		if (_S != NULL) delete _S;

		for (int k = 0; k < _I->nvehicles(); ++k) y[k].end();
		y.end();
	}
	catch (IloException& ex) {
		cout << "Error Branching: " << ex << endl;
	}
}

void MyCallback::collectMatchingCutsPerVehicles(CutPool& Cuts, CplexSolution* S)
{
	vector<int> K;
	for (int k = 0; k < _I->nvehicles(); ++k) K.push_back(k);

	for (auto k : K) {
		vector<int> Kp;
		Kp.push_back(k);
		auto X = S->getSetValues(k);
		sort(X.begin(), X.end(), [](double a, double b) {return a > b; });

		size_t n = Cuts.size();
		for (auto t : X) {
			collectMatchingCuts(Cuts, S, Kp, t);
			//if (n != Cuts.size()) break;
		}
	}
}

void MyCallback::collectMatchingCutsV2(CutPool& Cuts, CplexSolution* S, double target, const double threshold)
{
	

	for (int ks = 0; ks < _I->nvehicles(); ++ks) {

		vector<int> K;
		K.push_back(ks);

		auto X = S->getSetValues(ks);
		sort(X.begin(), X.end(), [](double a, double b) {return a > b; });
		for (auto t : X) {

			//We get the edges
			vector<edge> E;
			for (int i = 0; i < _I->nnodes(); ++i) for (int j = 0; j < i; j++) {
				double w = 0.0;
				for (int k : K) w += S->getXVal(i, j, k);
				if (w > EPS) E.push_back(edge(i, j, w));
			}

			vector<int> isconnected(_I->nnodes(), false);
			isconnected[S->initialdepotid()] = isconnected[S->finaldepotid()] = true;
			for (int s = 0; s < _I->nnodes(); ++s) if (!isconnected[s]) {
				
				//We get the connected component from s
				vector<int> H;
				vector<bool> visited(_I->nnodes(), false);
				visited[s] = !S->isdepot(s);

				bool stop = false;
				while (!stop) {
					stop = true;
					for (auto& e : E) if (visited[e.i] != visited[e.j] && e.w < t) {
						visited[e.i] = !S->isdepot(e.i);
						visited[e.j] = !S->isdepot(e.j);
						stop = !visited[e.i] || !visited[e.j];
					}
				}
				for (int i = 0; i < _I->nnodes(); ++i) if (visited[i]) {
					H.push_back(i);
					isconnected[i] = true;
				}

				//assert(count_if(H.begin(), H.end(), [&](int i) {return S->isdepot(i); }) == 0);

				//We get the arcs leaving H in the current Tree
				vector< edge > DeltaH;
				for (auto& e : E) if (visited[e.i] != visited[e.j]) DeltaH.push_back(e);

				if (DeltaH.empty()) continue;
				
				//We get the Teeth
				sort(DeltaH.begin(), DeltaH.end(), [](edge& a, edge& b) {return a.w < b.w; });
				vector< edge > T;
				T.push_back(DeltaH.back());
				DeltaH.pop_back();
				while (DeltaH.size() >= 2) {
					edge e1 = DeltaH.back(); DeltaH.pop_back();
					edge e2 = DeltaH.back(); DeltaH.pop_back();
					if (e1.w + e2.w - 1 < 0) break;
					else {
						T.push_back(e1);
						T.push_back(e2);
					}
				}

				//We repair the teeth
				unordered_set< int > VT;
				for (auto& e : T) {
					VT.insert(e.i);
					VT.insert(e.j);
				}
				for (auto i : VT) {
					auto n = count_if(T.begin(), T.end(), [i](edge& a) {return a.i == i || a.j == i; });
					//assert(n >= 1);
					if (n >= 2) {
						auto it = find(H.begin(), H.end(), i);
						if (it != H.end()) H.erase(it);
						else H.push_back(i);
						auto last = remove_if(T.begin(), T.end(), [i](edge& a) {return a.i == i || a.j == i; });
						assert(last != T.end());
						T.erase(last, T.end());
					}
				}


				if (H.size() < 2) continue;

				if (T.empty()) continue;
				if (T.size() % 2 == 0) T.erase(min_element(T.begin(), T.end(), [](edge& a, edge& b) {return a.w < b.w; }));
				if (T.size() >= 3) {
					vector< pair<int, int> >TE;
					for (auto& e : T) TE.push_back(pair<int, int>(e.i, e.j));

					bool added = Cuts.push(S->genMatchingCut(H, TE, K));
					//if (added) cout << "New Matching Cut Found " << threshold << endl;
				}
				else /*if (T.size() == 1 && count_if(H.begin(), H.end(), [&](int i) {return S->isdepot(i); }) < 2)*/ {
					//We add the GSECs
					//if(!Cuts.push(S->genCycleCut(H)))
						Cuts.push(S->genCycleCut(H, ks));
				}

				//if (find(H.begin(), H.end(), S->initialdepotid()) == H.end()) H.push_back(S->initialdepotid());
				//if (find(H.begin(), H.end(), S->finaldepotid()) == H.end()) H.push_back(S->finaldepotid());
				//if (S->score(H) <= target) {
				//	for (int k : K) Cuts.push(S->genPathCoverCut(H, k));
				//}
				//else for (int k : K) Cuts.push(S->genCycleCoverCut(H, k));


			}


		}


	}
}

void MyCallback::collectMatchingCuts(CutPool& Cuts, CplexSolution* S, vector<int> &K, const double threshold)
{
	vector<int> parent(_I->nnodes(), -1);
	vector<int> rank(_I->nnodes(), 1);

	

	vector<edge> E;
	for (int i = 0; i < _I->nnodes(); ++i) for (int j = 0; j < i; j++) {
		double w = 0.0;
		for (int k : K) w += S->getXVal(i, j, k);
		//assert(w <= 1.0 + EPS);
		if (w > EPS) E.push_back(edge(i, j, w));
	}
	sort(E.begin(), E.end(), [](edge& a, edge& b) {return a.w < b.w; });

	vector<edge> Tree;
	for (auto& currentedge : E) if (currentedge.w < threshold /* && !S->isdepot(currentedge.i) && !S->isdepot(currentedge.j) */ ) {
		if (findparent(currentedge.i, parent) != findparent(currentedge.j, parent)) {
			unite(currentedge.i, currentedge.j, parent, rank);
			Tree.push_back(currentedge);

			//We get the comb
			vector<int> H;
			vector<bool> visited(_I->nnodes(), false);
			visited[currentedge.i] = !S->isdepot(currentedge.i);
			visited[currentedge.j] = !S->isdepot(currentedge.j);

			bool stop = false;
			while (!stop) {
				stop = true;
				for (auto& e : Tree) if(visited[e.i]!=visited[e.j]){
					visited[e.i] = !S->isdepot(e.i);
					visited[e.j] = !S->isdepot(e.j);
					stop = stop && (!visited[e.i] || !visited[e.j]);
				}
			}
			for (int i = 0; i < _I->nnodes(); ++i) if (visited[i]) H.push_back(i);

			//We get the arcs leaving H in the current Tree
			vector< edge > DeltaH;
			for (auto& e : E) if (visited[e.i] != visited[e.j]) DeltaH.push_back(e);
			//copy_if(E.begin(), E.end(), back_inserter(DeltaH), [&](edge& e) {return visited[e.i] != visited[e.j]/* && e.w>=threshold*/; });

			if (DeltaH.empty()) continue;

			//We get the Teeth
			sort(DeltaH.begin(), DeltaH.end(), [](edge& a, edge& b) {return a.w < b.w; });
			vector< edge > T;
			T.push_back(DeltaH.back());
			DeltaH.pop_back();
			while (DeltaH.size() >= 2) {
				edge e1 = DeltaH.back(); DeltaH.pop_back();
				edge e2 = DeltaH.back(); DeltaH.pop_back();
				if (e1.w + e2.w - 1 < 0) break;
				else {
					T.push_back(e1);
					T.push_back(e2);
				}
			}

			//We repair the teeth
			vector<int> countocc(_I->nnodes(), 0);
			for (auto& e : T) {
				countocc[e.i]++;
				countocc[e.j]++;
			}
			for (int i = 0; i < _I->nnodes(); ++i) if(countocc[i]>=2){
				auto it = find(H.begin(), H.end(), i);
				if (it != H.end()) H.erase(it);
				else H.push_back(i);

				auto ite = T.begin();
				while (ite != T.end()) {
					if (ite->i == i || ite->j == i) ite = T.erase(ite);
					else ite++;
				}

			}

			auto c = count_if(H.begin(), H.end(), [currentedge](int i) {return currentedge.i == i || currentedge.j == i; });
			if (c != 2) continue;
			if (H.size() <= 2) continue;

			if (T.empty()) continue;
			if (T.size() % 2 == 0) T.erase(min_element(T.begin(), T.end(), [](edge& a, edge& b) {return a.w < b.w; }));
			if (T.size() >= 3) {
				vector< pair<int, int> >TE;
				for (auto& e : T) TE.push_back(pair<int, int>(e.i, e.j));
				Cuts.push(S->genMatchingCut(H, TE, K));
			}
			else if (T.size() == 1 && count_if(H.begin(), H.end(), [&](int i) {return S->isdepot(i); }) < 2) {
				//We add the GSECs
				Cuts.push(S->genCycleCut(H));
				//vector<int> Shore2;
				//for (int i = 0; i < _I->nnodes(); ++i) if (find(H.begin(), H.end(), i) == H.end()) Shore2.push_back(i);
				//for (auto s : H) if (!S->isdepot(s))
				//	Cuts.push_back(S->genFlowCut(H, Shore2, s));
			}

		}
	}
}

void MyCallback::collectTreeCuts(CutPool& Cuts, CplexSolution* S, vector<int>& K, double target)
{
	vector<int> parent(_I->nnodes(), -1);
	vector<int> rank(_I->nnodes(), 1);



	vector<edge> E;
	for (int i = 0; i < _I->nnodes(); ++i) for (int j = 0; j < i; j++) {
		double w = 0.0;
		for (int k : K) w += S->getXVal(i, j, k);
		//assert(w <= 1.0 + EPS);
		if (w > EPS) E.push_back(edge(i, j, w));
	}
	sort(E.begin(), E.end(), [](edge& a, edge& b) {return a.w > b.w; });

	vector<edge> Tree;
	for (auto& currentedge : E) if (!S->isdepot(currentedge.i) && !S->isdepot(currentedge.j)) {
		if (findparent(currentedge.i, parent) != findparent(currentedge.j, parent)) {
			unite(currentedge.i, currentedge.j, parent, rank);
			Tree.push_back(currentedge);

			//We get the comb
			vector<bool> visited(_I->nnodes(), false);
			visited[currentedge.i] = !S->isdepot(currentedge.i);
			visited[currentedge.j] = !S->isdepot(currentedge.j);

			bool stop = false;
			while (!stop) {
				stop = true;
				for (auto& e : Tree) if (visited[e.i] != visited[e.j]) {
					visited[e.i] = !S->isdepot(e.i);
					visited[e.j] = !S->isdepot(e.j);
					stop = stop && (!visited[e.i] || !visited[e.j]);
				}
			}

			vector<int> H;
			for (int i = 0; i < _I->nnodes(); ++i) if (visited[i]) H.push_back(i);
			vector<edge> subTree;
			for (auto& e : Tree) if (visited[e.i] && visited[e.j]) subTree.push_back(e);

			//for (auto& e : E) if (visited[e.i] != visited[e.j] && (S->isinitialdepot(e.i) || S->isinitialdepot(e.j))) {
			//	subTree.push_back(e);
			//	break;
			//}
			//for (auto& e : E) if (visited[e.i] != visited[e.j] && (S->isfinaldepot(e.i) || S->isfinaldepot(e.j))) {
			//	subTree.push_back(e);
			//	break;
			//}
			
			double L = 0.0;
			for (auto& e : subTree) L += _I->distance(_I->node(e.i), _I->node(e.j));
			

			

			double distO = DBL_MAX;
			double distD = DBL_MAX;
			Node* O = _I->node(S->initialdepotid());
			Node* D = _I->node(S->finaldepotid());
			for (int i : H) {
				distO = fmin(distO, _I->distance(O, _I->node(i)));
				distD = fmin(distD, _I->distance(D, _I->node(i)));
			}
			if (L + distO + distD <= _I->tmax()) {
				edge eO(0, 0, 0);
				edge eD(0, 0, 0);
				for (auto& e : Tree) if (visited[e.i] != visited[e.j] && S->isdepot(e.i) != S->isdepot(e.j)) {
					if (S->isinitialdepot(e.i) || S->isinitialdepot(e.j)) if (e.w > eO.w) eO = e;
					if (S->isfinaldepot(e.i) || S->isfinaldepot(e.j)) if (e.w > eD.w) eD = e;
				}
				distO = _I->distance(_I->node(eO.i), _I->node(eO.j));
				distD = _I->distance(_I->node(eD.i), _I->node(eD.j));
				if (eO.i != eO.j) subTree.push_back(eO);
				if (eD.i != eD.j) subTree.push_back(eD);
			}

			
			if (L + distO + distD > _I->tmax()/* || containsConflicts(H)*/) {
				IloExpr expr(_LMILP->_env);
				for(auto k : K) for(auto& e : subTree)
					expr += _LMILP->_x[_LMILP->getarcid(e.i,e.j,k)];


				IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, IloNum(subTree.size() - 1), LazyMILP::EdgeCut.c_str());
				expr.end();

				if (Cuts.push(R)) 
					cout << "EdgeCut added" << endl;
			}

			

			double score = S->score(H);
			if (S->score(H) <= target) {
				H.push_back(S->initialdepotid());
				H.push_back(S->finaldepotid());
				if (Cuts.push(S->genPathCoverCut(H))) 
					cout << "PCoverCut added" << endl;
			}
			else if (containsConflicts(H)) {
				if (Cuts.push(S->genPathCut(H)))
					cout << "PCut added" << endl;;
			}

		}
	}
}

void MyCallback::collectTreeCuts(CutPool& Cuts, CplexSolution* S, double target)
{
	vector<int> K;
	for (int k = 0; k < _I->nvehicles(); ++k) K.push_back(k);

	collectTreeCuts(Cuts, S, K, target);
}

double MyCallback::minSpanningTreeLength(const vector<int>& Nodes, const double threshold)
{
	vector<int> parent(_I->nnodes(), -1);
	vector<int> rank(_I->nnodes(), 1);

	

	vector<edge> E;
	for (int i : Nodes) for (int j : Nodes) if(i<j){
		double w = _I->distance(_I->node(i),_I->node(j));
		E.push_back(edge(i, j, w));
	}
	sort(E.begin(), E.end(), [](edge& a, edge& b) {return a.w < b.w; });

	vector<edge> Tree;
	for (auto& currentedge : E) {
		if (findparent(currentedge.i, parent) != findparent(currentedge.j, parent)) {
			unite(currentedge.i, currentedge.j, parent, rank);
			Tree.push_back(currentedge);
		}
	}

	return accumulate(Tree.begin(), Tree.end(), 0.0, [](double a, edge e) {return a + e.w; });
}

double MyCallback::minSpanningTreeLength(vector<edge>& E)
{
	vector<int> parent(_I->nnodes(), -1);
	vector<int> rank(_I->nnodes(), 1);

	sort(E.begin(), E.end(), [](edge& a, edge& b) {return a.w < b.w; });

	vector<edge> Tree;
	for (auto& currentedge : E) {
		if (findparent(currentedge.i, parent) != findparent(currentedge.j, parent)) {
			unite(currentedge.i, currentedge.j, parent, rank);
			Tree.push_back(currentedge);
		}
	}

	return accumulate(Tree.begin(), Tree.end(), 0.0, [](double a, edge e) {return a + e.w; });
}

void MyCallback::updatelog(const IloCplex::Callback::Context& context, CutPool& Cuts) {
	if (!_enablelog) return;
	CPXLONG THREADID = context.getLongInfo(IloCplex::Callback::Context::Info::ThreadId);
	assert(THREADID < _LMILP->_threads);
	auto L = &_Log[THREADID];
	for (auto R : Cuts) {
		string name = R->getName();
		auto it = find_if(L->begin(), L->end(), [name](pair<string, int> P) {return P.first == name; });
		if (it != L->end()) it->second++;
		else L->push_back(pair<string, int>(name, 1));
	}
}

// Constructor

MyCallback::MyCallback(LazyMILP* LMILP) : _LMILP(LMILP), _I(LMILP->_I) {

	for (int i = 0; i < _I->nnodes(); ++i) for (int j = i + 1; j < _I->nnodes(); ++j) {
		bool arecomp = false;
		for (int k = 0; k < _I->nvehicles() && !arecomp; ++k)
			arecomp = (_I->comp(k, i) && _I->comp(k, j));
		if (!arecomp) {
			assert(!_LMILP->isdepot(i) && !_LMILP->isdepot(j));
			_ConflictPairs.push_back(pair<int, int>(i, j));
		}
		else {
			Node* S = _I->node(0);
			Node* T = _I->node(_I->nnodes() - 1);
			Node* I = _I->node(i);
			Node* J = _I->node(j);
			double dij = _I->distance(I, J) + I->serviceTime() + J->serviceTime();
			double d1 = _I->distance(S, I) + dij + _I->distance(J, T);
			double d2 = _I->distance(S, J) + dij + _I->distance(I, T);
			if (d1 > _I->tmax() && d2 > _I->tmax())
				_ConflictPairs.push_back(pair<int, int>(i, j));
		}
	}

	_enablelog = true;
	_Log = new vector < pair<string, int> >[_LMILP->_threads];
}

void MyCallback::saveLog()
{
	// save statistics in a file
	string outputFile = "Output/LogBandC2.txt";

	vector<string> CutType;
	CutType.push_back(LazyMILP::CycleCut);
	CutType.push_back(LazyMILP::PathCut);
	CutType.push_back(LazyMILP::SubCycleCut);
	CutType.push_back(LazyMILP::CycleCoverCut);
	CutType.push_back(LazyMILP::CondCut);
	CutType.push_back(LazyMILP::LogCut);
	CutType.push_back(LazyMILP::FlowCut);
	CutType.push_back(LazyMILP::SECCut);
	CutType.push_back(LazyMILP::InCompCut);
	CutType.push_back(LazyMILP::MatchCut);
	CutType.push_back(LazyMILP::ScoreCut);
	CutType.push_back(LazyMILP::SubCycleCut);
	CutType.push_back(LazyMILP::InfPathCut);
	CutType.push_back(LazyMILP::PathCoverCut);
	CutType.push_back(LazyMILP::TmaxCut);
	CutType.push_back(LazyMILP::ScoreCut);
	CutType.push_back(LazyMILP::EdgeCut);
	CutType.push_back(LazyMILP::ConnectCut);





	int w = 12;


	fstream output;
	output.open(outputFile.c_str(), ios::in);
	output.setf(ios::left);
	output.precision(2);
	output.setf(ios::fixed, ios::floatfield);

	if (!output.is_open()) {
		ofstream output(outputFile.c_str());
		output << setw(w) << "Instance" << setw(w) << "ObjF" << setw(w) << "Status" << setw(w) << "Threads";
		for (auto& s : CutType) output << setw(w + 5) << s;
		output << endl;
		output.close();
	}
	else output.close();
	

	output.setf(ios::left);
	output.precision(2);
	output.setf(ios::fixed, ios::floatfield);
	output=fstream(outputFile.c_str(), ios::app);
	string name = _I->name();
	size_t lastSeparator = name.find_last_of("/\\");
	if (lastSeparator != std::string::npos) 
		name=name.substr(lastSeparator + 1);
	
	output << setw(w) << name << setw(w) << _LMILP->_OF << setw(w) << _LMILP->_cplex.getStatus() << setw(w) << _LMILP->_threads;
	for (auto& s : CutType) {
		int n = 0;
		for (int t = 0; t < _LMILP->_threads; ++t) 
			for (auto& L : _Log[t]) if (L.first == s) n += L.second;
		output << setw(w + 5) << n;
	}
	output << endl;
	
	output.close();
}

void MyCallback::invoke(const IloCplex::Callback::Context& context)
{
	
	addLazyCuts(context);

	addUserCuts(context);
		
	//searchSolutionFromLP(context);

	addBranches(context);
	
}
