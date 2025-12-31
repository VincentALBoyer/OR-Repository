#include "BandC.h"

const string BandC::CycleCut = "CycleCut";
const string BandC::PathCut = "PathCut";
const string BandC::ConflictPathCut = "ConflictPathCut"; 
const string BandC::CycleCoverCut = "CycleCoverCut";
const string BandC::CycleCoverCut2 = "CycleCoverCut(2)";
const string BandC::CondCut = "CondCut";
const string BandC::LogCut = "LogCut"; 
const string BandC::LogCompCut = "LogCompCut";
const string BandC::FlowCut = "FlowCut";
const string BandC::InCompCut = "InCompCut";
const string BandC::InfPathCut = "InfPathCut";
const string BandC::MatchCut = "MatchCut"; 
const string BandC::SECCut = "SECCut";
const string BandC::UnknownCut = "UnknownCut";


void BandC::solvemethod(Solution* S)
{
	//MILPStart
	if (S != NULL) {

		IloNumVarArray Var(_env);
		IloNumArray Val(_env);

		double P = 0;
		int i = S->front();
		assert(_I->Node(i)->isInitialDepot());
		for (auto j : *S) if (i != j) {
			Var.add(_x[getarcid(i, j)]); Val.add(1.0);
			if (!_I->Node(j)->isFinalDepot()) Var.add(_y[j]); Val.add(1);
			i = j;
		}
		


		_cplex.addMIPStart(Var, Val, IloCplex::MIPStartRepair);

		Var.end();
		Val.end();
	}

	_cplex.setParam(IloCplex::TiLim, _timlim);
	_cplex.setParam(IloCplex::EpGap, _mingap);
	_cplex.setParam(IloCplex::Threads, _threads); //Sensible parameter

	
	try {


		BandCCallBack CB(this);

		_cplex.use(&CB, IloCplex::Callback::Context::Id::Candidate | IloCplex::Callback::Context::Id::Relaxation /*| IloCplex::Callback::Context::Id::Branching*/);

		_cplex.solve();

		CB.printlog();

		CB.savelog();

	}
	catch (IloException& ex) {
		cout << "Error: " << ex << endl;
	}
}

BandC::BandC(Instance* I) : Solver(I, SolveMethod::MILP_BC){

	//Undirected Arcs
	int aid = 0;
	for (int i = 0; i < _I->nnodes(); ++i) {
		for (int j = 0; j < i; ++j) {
			_Edges.push_back( new arc(i, j, aid));
			assert(getarcid(i, j) == aid);
			aid++;
		}
	}
	

	try {
		_model = IloModel(_env);
		_cplex = IloCplex(_model);
		_Constraints = IloRangeArray(_env);
		_obj = IloAdd(_model, IloMaximize(_env, 0));

		/* ======= Decision variables ======= */
		_y = IloNumVarArray(_env);
		for (int i = 0; i < _I->nnodes(); i++) {
			string label = "Y(" + to_string(i) + ")";
			if (_I->Node(i)->isDepot() || _I->Node(i)->isMandatoryNode())
				_y.add(IloNumVar(_env, 1, 1, ILOINT, label.c_str()));
			else if(_I->Node(i)->isFeasible())
				_y.add(IloNumVar(_env, 0, 1, ILOINT, label.c_str()));
			else
				_y.add(IloNumVar(_env, 0, 0, ILOINT, label.c_str()));
		}

		_x = IloNumVarArray(_env);
		for (auto e : _Edges) {
			assert(e != NULL);
			assert(e->id() == (int)_x.getSize());
			string label = "X(" + to_string(e->geti()) + "," + to_string(e->getj()) + ")";
			_x.add(IloNumVar(_env, 0, 1, ILOBOOL, label.c_str()));
		}
		
		/* ======= Objective ======= */
		IloExpr obj(_env);
		for (int i = 0; i < _I->nnodes(); ++i)
			obj += _I->Node(i)->score() * _y[i];
		_obj.setExpr(obj);
		obj.end();

		/* ======= Constraints ======= */
		// Connectivity
		for (int i = 0; i < _I->nnodes(); i++) {
			
			IloExpr conec(_env);
			auto Ei = getConnectedArc(i);
			for (auto e : Ei)  conec += _x[e->id()];

			string label1 = "c3_" + to_string(i);
			int degree = 2;
			if (_I->Node(i)->isDepot()) degree = 1;
			_Constraints.add(IloRange(_env, 0, conec - degree * _y[i], 0, label1.c_str()));

			conec.end();
		}

		//Conflicts
		for (int i = 0; i < _I->nnodes(); ++i) {
			node* N = _I->Node(i);
			auto C = N->getConflictList();
			if (!C.empty()) {
				assert(!N->isInitialDepot() && !N->isFinalDepot());
				for (auto Nj : C) {
					int j = Nj->id();
					IloExpr expr(_env);
					string label = "C_7_" + to_string(i) + "_" + to_string(j);
					expr = _y[i] + _y[j];
					_Constraints.add(IloRange(_env, -IloInfinity, expr, 1, label.c_str()));
					expr.end();
				}
			}
		}
		

		// Time budget
		IloExpr travelTime(_env);
		for (auto e : _Edges) {
			node* O = _I->Node(e->geti());
			node* D = _I->Node(e->getj());
			travelTime += _I->travellingtime(O, D) * _x[e->id()];
			assert(fabs(_I->travellingtime(O, D) - _I->travellingtime(D, O)) < 0.001);
		}
		string label = "c6";
		_Constraints.add(IloRange(_env, -IloInfinity, travelTime, _I->Tmax(), label.c_str()));
		travelTime.end();

		//Logical cuts 
		//for (int i = 0; i < _I->nnodes(); i++) if (!_I->Node(i)->isDepot()) {
		//	auto Ei = getConnectedArc(i);
		//	for (auto e : Ei) {
		//		_Constraints.add(_x[e->id()] - _y[i] <= 0);
		//	}
		//}

		//An other cuts since we have mandatory nodes
		IloExpr expr(_env);
		int n = 0;
		for (int i = 0; i < _I->nnodes(); ++i) {
			if (_I->Node(i)->isMandatoryNode()) n++;
			expr -= _y[i];
		}
		for (auto e : _Edges) expr += _x[e->id()];
		_Constraints.add(expr <= -1);
		expr.end();

		// Checking for infeasible nodes (Preprocessing)
		node* S = _I->InitialDepot();
		node* T = _I->FinalDepot();
		for (int i = 0; i < _I->nnodes(); i++) {
			node* I = _I->Node(i);
			for (int j = 0; j < i; j++) {
				node* J = _I->Node(j);
				double dij = _I->travellingtime(I, J) ;
				double d1 = _I->travellingtime(S, I) + dij + _I->travellingtime(J, T);
				double d2 = _I->travellingtime(S, J) + dij + _I->travellingtime(I, T);


				if (d1 > _I->Tmax() && d2 > _I->Tmax()) {
					_x[getarcid(i, j)].setUB(0.0);
				}
				if (_I->travellingtime(S, I) + _I->travellingtime(I, T) > _I->Tmax())
					_y[i].setUB(0.0);

				if (_I->travellingtime(S, J) + _I->travellingtime(J, T) > _I->Tmax())
					_y[j].setUB(0.0);

			}
			auto C = I->getConflictList();
			auto n = count_if(C.begin(), C.end(), [&](node* N) {return N->isMandatoryNode(); });
			if(n>0) _y[i].setUB(0.0);
			assert(n == 0 || !I->isMandatoryNode());

		}

		// Checking for unfeasible arcs (Preprocessing)
		for (int i = 0; i < _I->nnodes(); i++) {
			node* I = _I->Node(i);
			for (int j = 0; j < i; j++) {
				node* J = _I->Node(j);
				if (!_I->areCompatible(I, J)) {
					_x[getarcid(i, j)].setUB(0.0);
				}
			}
		}

		_model.add(_Constraints);
		//_cplex.exportModel("Model.lp");
		_Constraints.end();
	}

	catch (IloException& e) {
		cout << e << endl;
		exit(1000);
	}
}

BandC::~BandC()
{
	_env.end();
	while (!_Edges.empty()) {
		auto e = _Edges.back();
		_Edges.pop_back();
		delete e;
	}
}

Solution* BandC::recoversolution()
{
	Solution* S = NULL;
	try {
		cout << "Cplex Status: " << _cplex.getStatus() << endl;
		if (_cplex.getStatus() == IloAlgorithm::Feasible || _cplex.getStatus() == IloAlgorithm::Optimal) {

			S = new Solution(_I);

			node* Ni = _I->InitialDepot();
			S->push_back(Ni->id());
			while(!_I->Node(S->back())->isFinalDepot()) {
				int i = S->back();
				for (int j = 0; j < _I->nnodes(); ++j) if (j != i && find(S->begin(),S->end(),j)==S->end()) {
					assert(!Ni->isFinalDepot());
					if (_cplex.getValue(_x[getarcid(i,j)]) >= 0.5) {
						S->push_back(j);
						break;
					}

				}
				assert(i!=S->back());

			}

			cout << "Fitness: " << S->fitness() << endl;
			cout << "Cplex Obj: " << _cplex.getObjValue() << endl;
			cout << "Gap: " << _cplex.getMIPRelativeGap() << endl;
			S->print();

		}


	}
	catch (IloException& e) {
		cout << e << endl;
		exit(1000);
	}
	return S;
}


vector<int> LRSolution::getRandomPath(int s, double threshold)
{
	int n = _I->nnodes();

	if (s < 0) {
		IloNumArray YVal = getYVal();

		vector<int> sources;
		for (int i = 0; i < n; ++i) if (!isdepot(i) && YVal[i] > threshold)
			sources.push_back(i);



		if (sources.empty()) sources.push_back(initialdepotid());
		s = *RouletteWheelSelection(sources.begin(), sources.end(), [&](int i) {return  round(YVal[i] * 100);  });
		YVal.end();
	}


	vector<int> Path;
	Path.push_back(s);

	extendRandomPath(Path, threshold);
	reverse(Path.begin(), Path.end());
	if (!isdepot(s)) extendRandomPath(Path, threshold);

	if (isinitialdepot(Path.back()) || isfinaldepot(Path.front())) reverse(Path.begin(), Path.end());

	if (isinitialdepot(Path.front()) && !isfinaldepot(Path.back())) Path.push_back(finaldepotid());
	else if (!isinitialdepot(Path.front()) && isfinaldepot(Path.back())) Path.insert(Path.begin(), initialdepotid());
	else if (!isinitialdepot(Path.front()) && !isfinaldepot(Path.back())) {
		node* S = _I->Node(initialdepotid());
		node* T = _I->Node(finaldepotid());
		node* B = _I->Node(Path.back());
		node* F = _I->Node(Path.front());
		double d_st = _I->travellingtime(S, F) + _I->travellingtime(B, T);
		double d_ts = _I->travellingtime(B, S) + _I->travellingtime(T, F);
		if (d_ts < d_st) reverse(Path.begin(), Path.end());
		Path.insert(Path.begin(), initialdepotid());
		Path.push_back(finaldepotid());
	}

	assert(!Path.empty());

	return Path;

}

vector<int> LRSolution::getRandomInfPath(int s)
{

	double l = 0.0;
	double sum = 0.0;
	vector<int> Path;
	Path.push_back(s);

	vector<bool> _RStatus(nnodes(), false);
	_RStatus[s] = true;
	_RStatus[initialdepotid()] = true;
	_RStatus[finaldepotid()] = true;
	do {
		int i = Path.back();
		_RStatus[i] = true;

		vector<int> Candidates;
		for (int j = 0; j < nnodes(); ++j) if (!_RStatus[j] && getXVal(i, j)>EPS && sum + getXVal(i,j) >= Path.size() - 1)
			Candidates.push_back(j);


		int j = finaldepotid();
		if (!Candidates.empty()) {
			j = *RouletteWheelSelection(Candidates.begin(), Candidates.end(), [&](int i) {return  round(getXVal(i, j) * 100);  });
			Path.push_back(j);
			sum += getXVal(i, j);
			l += _I->travellingtime(_I->Node(i), _I->Node(j)) ;
		}
		else {
			break;
		}
	} while (l <= _I->Tmax());

	assert(!isdepot(Path.front()));
	assert(!isdepot(Path.back()));

	//We manually add the depot to minimize the total time
	node* S = _I->InitialDepot();
	node* T = _I->FinalDepot();
	node* B = _I->Node(Path.back());
	node* F = _I->Node(Path.front());
	double d_st = _I->travellingtime(S, F) + _I->travellingtime(B, T);
	double d_ts = _I->travellingtime(B, S) + _I->travellingtime(T, F);
	if (d_ts < d_st) reverse(Path.begin(), Path.end());
	Path.insert(Path.begin(), initialdepotid());
	Path.push_back(finaldepotid());


	return Path;
}

vector<int> LRSolution::getHeuristicRoute()
{
	return getRandomPath();
}


vector<int> LRSolution::bfs(double** RNetwork, int n, int s, int t, double threshold) {

	vector<int> S; S.push_back(s);
	vector<int> T;
	if (t >= 0) T.push_back(t);
	else {
		T.push_back(initialdepotid());
		T.push_back(finaldepotid());
	}
	return bfs(RNetwork, n, S, T, threshold);

}

vector<int> LRSolution::bfs(double** RNetwork, int n, vector<int>& S, vector<int>& T, double threshold)
{
	vector<int> Path;

	assert(RNetwork != NULL);
	vector<int> P(n, -2);

	for (auto t : T) P[t] = -3;

	vector<int> Q;
	for (auto s : S) {
		Q.push_back(s);
		P[s] = -1;
	}


	while (!Q.empty()) {
		int i = Q.back();
		Q.pop_back();
		for (int j = 0; j < n; ++j) if (P[j] <= -2 && RNetwork[i][j] > threshold) {
			Q.push_back(j);
			bool targetfound = (P[j] == -3);
			P[j] = i;
			if (targetfound) {
				Q.clear();

				for (int k = j; k >= 0; k = P[k]) Path.push_back(k);

				assert(Path.size() > 1 && P[Path.back()] == -1);

				break;
			}
		}
	}


	return Path;
}

vector<int> LRSolution::strongbfs(double** RNetwork, int n, vector<int>& S, vector<int>& T, double threshold)
{
	vector<int> Path;

	assert(RNetwork != NULL);
	vector<int> P(n, -2);
	vector<double> Eval(n, 1);

	for (auto t : T) P[t] = -3;

	vector<int> Q;
	for (auto s : S) {
		Q.push_back(s);
		P[s] = -1;
	}


	while (!Q.empty()) {
		auto iti = max_element(Q.begin(), Q.end(), [Eval](int i, int j) {return Eval[i] < Eval[j]; });
		int i = *iti;
		Q.erase(iti);
		for (int j = 0; j < n; ++j) if (P[j] <= -2 && RNetwork[i][j] > threshold) {
			
			Eval[j] = Eval[i] + RNetwork[i][j] - 1;
			if (Eval[j] < EPS) continue;
			bool targetfound = (P[j] == -3);
			P[j] = i;
			if (targetfound && Eval[j] > EPS) {
				Q.clear();

				for (int k = j; k >= 0; k = P[k]) Path.push_back(k);

				assert(Path.size() > 1 && P[Path.back()] == -1);

				break;
			}
			else if(!targetfound) Q.push_back(j);
		}
	}


	return Path;
}

pair< vector<int>, vector<int> > LRSolution::getShoresMinCut(int s, int t)
{
	vector<int> S; S.push_back(s);
	vector<int> T;
	if (t >= 0) T.push_back(t);
	else {
		T.push_back(initialdepotid());
		T.push_back(finaldepotid());
	}
	return getShoresMinCut(S, T);

}

pair<vector<int>, vector<int>> LRSolution::getShoresMinCut(vector<int>& S, vector<int>& T)
{
	int n = _I->nnodes();
	double** RNetwork = extractNetwork();
	

	double maxflow = 0.0;
	vector<int> path = bfs(RNetwork, n, S, T);
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

	//Then we get the shores
	vector<int> shore1; // Reachable shore
	vector<int> shore2; // Unreachable shore

	bool* _RStatus = new bool[n];
	for (int i = 0; i < n; ++i) _RStatus[i] = false;
	stack<int> Q;
	for (auto s : S) {
		Q.push(s);
		_RStatus[s] = true;
	}

	while (!Q.empty()) {
		int i = Q.top();
		Q.pop();
		for (int j = 0; j < n; ++j) if (!_RStatus[j] && RNetwork[i][j] > EPS) {
			Q.push(j);
			_RStatus[j] = true;
		}
	}

	for (int i = 0; i < n; ++i) {
		if (_RStatus[i])  shore1.push_back(i); // Node is reachable, add to shore1
		else shore2.push_back(i); // Node is not reachable, add to shore2
	}

	
	delete[] _RStatus;

	for (int i = 0; i < n; ++i) delete[] RNetwork[i];
	delete[] RNetwork;


	return pair< vector<int>, vector<int> >(shore1, shore2);
}

vector<int> LRSolution::getRoute() {
	
	return getRandomPath(initialdepotid());
}


void LRSolution::repairroute(vector<int>& R, double MinScore) {

	assert(R.empty() || (isinitialdepot(R.front()) && isfinaldepot(R.back())));

	//First we remove all incompatible nodes
	auto C = getConflictNodes(R);
	for (auto& c : C) {
		int i = c.first;
		int j = c.second;
		if (score(i) < score(j)) swap(i, j);
		auto iti = find(R.begin(), R.end(), i);
		auto itj = find(R.begin(), R.end(), j);
		if (iti != R.end() && itj != R.end()) {
			if (_I->Node(i)->isMandatoryNode()) R.erase(itj);
			else if (_I->Node(j)->isMandatoryNode()) R.erase(iti);
			else R.erase(itj);
		}
	}

	//Then we try to reduce the route length
	adjustRoutesLength(R);

}

void LRSolution::adjustRoutesLength(vector<int>& R)
{
	double* eval = new double[nnodes()];
	

	//We try to reduce the route length
	double length = this->length(R);

	
	while (length > _I->Tmax()) {
		for (auto it = R.begin(); it != R.end(); it++) {
			if (it == R.begin() || it + 1 == R.end()) eval[*it] = -INFINITY;
			else {
				int i = *(it - 1);
				int j = *it;
				int k = *(it + 1);
				double dij = _I->travellingtime(_I->Node(i), _I->Node(j));
				double djk = _I->travellingtime(_I->Node(j), _I->Node(k));
				double dik = _I->travellingtime(_I->Node(i), _I->Node(k));
				eval[j] = (dij + djk - dik);
			}
		}
		auto bestit = max_element(R.begin(), R.end(), [&](int i, int j) {return eval[i] / (double)_I->Node(i)->score() < eval[j] / (double)_I->Node(j)->score(); });
		length -= eval[*bestit];
		R.erase(bestit);
	}

	assert(R.empty() || (isinitialdepot(R.front()) && isfinaldepot(R.back())));
	
	delete[] eval;
}

void LRSolution::optimizeRoutesLength(vector<int>& R)
{
	bool improved = true;
	while (improved) {
		improved = false;
		double length = this->length(R);;
		for (auto iti = R.begin(); iti != R.end() && !improved; ++iti) if (!isdepot(*iti)) {
			for (auto itj = iti + 1; itj != R.end() && !improved; ++itj) if (!isdepot(*itj)) {

				node* prevI = _I->Node(*(iti - 1));
				node* I = _I->Node(*iti);
				node* nextI = _I->Node(*(iti + 1));

				node* prevJ = _I->Node(*(itj - 1));
				node* J = _I->Node(*itj);
				node* nextJ = _I->Node(*(itj + 1));

				double delta = 0;
				delta -= _I->travellingtime(prevI, I) ;
				if (nextI != J) delta -= _I->travellingtime(I, nextI) + _I->travellingtime(prevJ, J);
				delta -= _I->travellingtime(J, nextJ);
				delta += _I->travellingtime(prevI, J) + _I->travellingtime(J, nextI);
				delta += _I->travellingtime(prevJ, I) + _I->travellingtime(I, nextJ);

				if (delta < -EPS) {
					std::swap(*iti, *itj);
					length +=delta;

					improved = true;
				}
			}
		}
	}



	
}

void LRSolution::improveRoutes(vector<int>& R)
{


	vector<int> S;
	vector<bool> isCandidate(nnodes(), true);
	for (int i = 0; i < _I->nnodes(); ++i) if (!isdepot(i)) S.push_back(i);
	for (int i : R) {
		isCandidate[i] = false;
		auto C = _I->Node(i)->getConflictList();
		for (auto N : C) isCandidate[N->id()] = false;
	}

	sort(S.begin(), S.end(), [&](int i, int j) { return _I->Node(i)->score() > _I->Node(j)->score(); });

	//First the Mandatory
	for (auto i : S) if (_I->Node(i)->isMandatoryNode() && isCandidate[i]) {
		isCandidate[i] = false;
		if (!bestinsert(R, i) && !swapinsert(R, i)) return; //Infeasible path
	}

	//Then the optionals
	for (auto i : S) if (_I->Node(i)->isOptionalNode() && isCandidate[i]) {
		bestinsert(R, i);
		auto C = _I->Node(i)->getConflictList();
		for (auto N : C) isCandidate[N->id()] = false;
	}

}

pair<IloNumVarArray*, IloNumArray*> LRSolution::extractSolution()
{
	IloNumVarArray* X = new IloNumVarArray(_LMILP->_env);
	IloNumArray* XVal = new IloNumArray(_LMILP->_env);


	vector<double>  YVal(nnodes(),0);

	for (auto e : _LMILP->_Edges) {
		assert(e->id() == _LMILP->getarcid(e->geti(), e->getj()));
		IloNum val = _XVal[e->id()];
		X->add(_LMILP->_x[e->id()]);
		XVal->add(val);
		if (val > 0.5) {
			YVal[e->geti()] = YVal[e->getj()] = 1;
		}
	}

	for (int i = 0; i < nnodes(); ++i) {
		X->add(_LMILP->_y[i]); 
		XVal->add(YVal[i]);
		if (YVal[i] < 1.0 - EPS) assert(_I->Node(i)->isOptionalNode() || !_I->Node(i)->isFeasible());
	}

	std::pair< IloNumVarArray*, IloNumArray* > P(X, XVal);

	return P;
}

void LRSolution::exportSubGraph(const string& filename, vector<int>& S,int level)
{
	//Levels: 
	//			0->only nodes in S, 
	//			1->Nodes in S and nodes connected to S
	//			2->All nodes

	vector<bool> inS(nnodes(), false);
	for (int i : S) inS[i] = true;

	vector<int> Nodes;
	for (int i = 0; i < nnodes(); ++i) {
		if (inS[i]) Nodes.push_back(i);
		else if (level == 1 && !inS[i]) {
			bool hasEdge = false;
			for (int j : S) if (getXVal(i, j) > EPS) {
				hasEdge = true;
				break;
			}
			if (hasEdge) Nodes.push_back(i);
		}
		else if (level == 2) {
			//We check if there is at least one edge
			bool hasEdge = false;
			for (int j = 0; j < _I->nnodes(); j++) if (i!=j && getXVal(i, j) > EPS) {
				hasEdge = true;
				break;
			}
			if(hasEdge) Nodes.push_back(i);
		}
	}

	// Export the subgraph in dot format
	ofstream file(filename);
	file << "graph G {" << endl;

	//Title has the instance name
	file << "label=\"" << _I->instancename() << "\"" << endl;

	//Nodes in S
	for (int i :S) {
		if (_I->Node(i)->isDepot())
			file << i << " [label=\"" << i << "\", pos=\"" << _I->Node(i)->x() << "," << _I->Node(i)->y() << "!\", style=filled, fillcolor=red]" << endl;
        else
            file << i << " [label=\"" << i << "\", xlabel=\"" << _I->Node(i)->score() << "\", pos=\"" << _I->Node(i)->x() << "," << _I->Node(i)->y() << "!\"]" << endl;
	}

	//Nodes not in S
	for (int i : Nodes) if (!inS[i]) {
		if (_I->Node(i)->isDepot())
			file << i << " [label=\"" << i << "\", pos=\"" << _I->Node(i)->x() << "," << _I->Node(i)->y() << "!\", style=filled, fillcolor=red]" << endl;
        else 
			file << i << " [label=\"" << i << "\", xlabel=\"" << _I->Node(i)->score() << "\", pos=\"" << _I->Node(i)->x() << "," << _I->Node(i)->y() << "!\", style=dotted]" << endl;
	}

	//Edges in S
	for (int i : S)  {
		for (int j :S){
			if (i < j && getXVal(i, j) > EPS) {
                file << i << " -- " << j << " [label=\"" << getXVal(i, j) << "\"]" << endl;
			}
		}
	}

	//Edges not in S
	for (int i : Nodes) {
		for (int j :Nodes) if (i<j && (!inS[i] || !inS[j]) ) {
			if ( getXVal(i, j) > EPS) {
                file << i << " -- " << j << " [style=dotted, label=\"" << getXVal(i, j) << "\"]" << endl;
			}
		}
	}
	file << "}" << endl;
}


void BandCCallBack::addLazyCuts(const IloCplex::Callback::Context& context)
{
	if (!context.inCandidate()) return;

	try {
		LRSolution* _S = new LRSolution(_LMILP, [&](IloNumVarArray& X, IloNumArray& V) { context.getCandidatePoint(X, V); });

		double target = context.getIncumbentObjective();
		double obj = context.getCandidateObjective();

		//We get the Route
		vector<int>  Route= _S->getRandomPath(_S->initialdepotid());
		assert(_S->isinitialdepot(Route.front()));
		assert(_S->isfinaldepot(Route.back()));		

		//We check if it is an acceptable solution
		double fitness = _S->fitness(Route);
		if (fitness >= obj - EPS) {
			cout << "* Integer Solution Accepted..." << obj << " * " << endl;
			return;
		}
		else cout << "Integer Solution Rejected..." << obj << " (" << fitness << ")" << endl;

		CutPool Cuts([&](IloExpr& E) {return context.getCandidateValue(E); }, 0.0001);

		//Routes Cuts
		vector< vector<int> > Routes;
		Routes.push_back(Route);
		collectCutsOnRoutes(Cuts, Routes, _S, target);

		//Flow Cuts
		collectCutsOnFlow(Cuts, _S, target);

		updatelog(context, Cuts);

		IloRangeArray LazyCuts(context.getEnv());

		for (auto Cut : Cuts) {
			LazyCuts.add(*Cut);
		}

		if (LazyCuts.getSize() == 0) {
			for (int i = 0; i < _I->nnodes(); ++i)
				for (int j = 0; j < i; j++)
					if (_S->getXVal(i, j) > 0.5)
						cout << i << " -- " << j << endl;
		}

		assert(LazyCuts.getSize() > 0);
		context.rejectCandidate(LazyCuts);
		LazyCuts.end();

		Cuts.clean();

		delete _S;
	}
	catch (IloException& ex) {
		cerr << "Error: " << ex << endl;
	}

}

void BandCCallBack::addUserCuts(const IloCplex::Callback::Context& context)
{
	
	static double _BOUND = DBL_MAX;
	static int _ITER = 0;
	if (!context.inRelaxation()) return ;
	CPXINT AFTERCUTLOOP = context.getIntInfo(IloCplex::Callback::Context::Info::AfterCutLoop);

	CPXLONG THREADID = context.getIntInfo(IloCplex::Callback::Context::Info::ThreadId);

	CPXLONG Node_LEFT = context.getLongInfo(IloCplex::Callback::Context::Info::NodesLeft);
	if (Node_LEFT > 1 && THREADID==0) _ITER = 0;
	if (Node_LEFT > 1 /*&& RAND01() > 0.01*/) return ;
	if (Node_LEFT == 1 && THREADID == 0) {
		double b = context.getRelaxationObjective();
		if (fabs(b - _BOUND)/b < 0.001) _ITER++;
		else {
			_BOUND = b;
			_ITER = 0;
		}
		if (_ITER > 10) {
			cout << "Max Iteration Limit Reached..." << endl;
			return ;
		}
	}

	try {

		LRSolution* _S = new LRSolution(_LMILP, [&context](IloNumVarArray& X, IloNumArray& V) { context.getRelaxationPoint(X, V); });

		double target = context.getIncumbentObjective();

		CutPool Cuts([&](IloExpr& E) {return context.getRelaxationValue(E); });

		size_t NMinCuts = SIZE_MAX;

	
		collectLogicalCuts(Cuts, _S);

		//Matching Cuts
		if (Cuts.size()<NMinCuts)
			collectMatchingCuts(Cuts, _S);

		//Flow Cuts
		if (Cuts.size() < NMinCuts)
			collectCutsOnFlow(Cuts, _S, target, EPS);

		
		//Routes Cuts
		if (Cuts.size() < NMinCuts) {
			//We get Some Routes
			vector< vector<int> > Routes;

			vector<bool> isCandidate(_I->nnodes(), true);
			isCandidate[_S->initialdepotid()] = isCandidate[_S->finaldepotid()] = false;
			for (int i = 0; i < _I->nnodes(); ++i) if (isCandidate[i]) {
				auto Path = _S->getRandomInfPath(i);
				if(Path.size()>3) Routes.push_back(Path);
				for (auto s : Path) isCandidate[s] = false;
			}
			collectCutsOnRoutes(Cuts, Routes, _S, target);
		}


		updatelog(context, Cuts);

		IloCplex::CutManagement purgeable = IloCplex::UseCutForce;

		for(auto R : Cuts) context.addUserCut(*R, purgeable, false);

		Cuts.clean();

		delete _S;

	}
	catch (IloException& ex) {
		cerr << "Error: " << ex << endl;
	}


}

void BandCCallBack::searchSolutionFromLP(const IloCplex::Callback::Context& context)
{
	LRSolution* _S = NULL;
	if (context.inCandidate())
		_S = new LRSolution(_LMILP, [&context](IloNumVarArray& X, IloNumArray& V) { context.getCandidatePoint(X, V); });
	else if (context.inRelaxation()) {

		CPXLONG Node_LEFT = context.getLongInfo(IloCplex::Callback::Context::Info::NodesLeft);
		if (Node_LEFT > 1 && RAND01() > 0.1) return;
		_S = new LRSolution(_LMILP, [&context](IloNumVarArray& X, IloNumArray& V) { context.getRelaxationPoint(X, V); });

	}
	else return;

	auto Route = _S->getHeuristicRoute();

	_S->repairroute(Route);

	_S->optimizeRoutesLength(Route);

	_S->improveRoutes(Route);

	
	if (_S->isFeasible(Route)) {
		_S->reset();
		int i = Route.front();
		for (auto j : Route) if (j != i) {
			_S->setXVal(i, j, 1.0);
			i = j;
		}

		if (context.inCandidate()) cout << "Heuristic Solution from Candidate: " << _S->fitness(Route) << endl;
		
		addSolution(context, _S);
	}

	delete _S;

}

bool BandCCallBack::addSolution(const IloCplex::Callback::Context& context, LRSolution* S)
{
	
	double target = context.getIncumbentObjective();
	
	auto R = S->getRoute();
	if (S->length(R) > _I->Tmax()) 
		return false;
	double fitness = S->fitness(R);

	bool add = fitness > target + EPS;
	add = add && (!context.inCandidate() || fabs(fitness - context.getCandidateObjective()) > EPS);
	if (!add) return add;

	cout << "Passing New solution from Heuristic: " << fitness << " (";
	if (context.inRelaxation()) cout << context.getRelaxationObjective() << ") LR" << endl;
	else if (context.inCandidate()) cout << context.getCandidateObjective() << ") C" << endl;

	try {

		auto P = S->extractSolution();

		context.postHeuristicSolution(*(P.first), *(P.second), fitness, IloCplex::Callback::Context::SolutionStrategy::CheckFeasible);
		
		P.first->end(); delete P.first;
		P.second->end(); delete P.second;
	}
	catch (IloException& e) {
		cout << "Error: " << e << endl;
	}


	return add;
}

bool BandCCallBack::isvalidcut(IloRange* R, const std::function<IloNum(IloExpr&)>& getValue) {
	if (R == NULL) return false;
	IloExpr expr = R->getExpr();
	IloNum val = getValue(expr);
	IloNum lhs = R->getLB();
	IloNum rhs = R->getUB();

	IloNum threshold(EPS);

	expr.end();
	return lhs - val > threshold || val - rhs > threshold;
}

vector<IloRange*> BandCCallBack::collectLogicalCuts(CutPool& Cuts, LRSolution* S)
{
	IloNumArray Y = S->getYVal();
	
	for (auto& e : _LMILP->_Edges) if (S->getXVal(e->id()) > EPS && _LMILP->timeLeft() > 0) {
		if (!_I->Node(e->geti())->isDepot()) Cuts.push(S->genLogicalConstraint(e->geti(), e->getj()));
		if (!_I->Node(e->getj())->isDepot()) Cuts.push(S->genLogicalConstraint(e->getj(), e->geti()));
	}

	//The conflicts
	vector<int> nodes;
	for (int i = 0; i < _I->nnodes(); i++) if (Y[i] > EPS) nodes.push_back(i);
	for (auto& C : _ConflictPairs) if (Y[C.first] > EPS && Y[C.second] > EPS && _LMILP->timeLeft() > 0) {
		int nodefirst = *max_element(nodes.begin(), nodes.end(), [&](int i, int j) {return S->getXVal(C.first, i) < S->getXVal(C.first, j); });
		int nodesecond = *max_element(nodes.begin(), nodes.end(), [&](int i, int j) {return S->getXVal(C.second, i) < S->getXVal(C.second, j); });
		
		int edgefirst = _LMILP->getarcid(C.first, nodefirst);
		int edgesecond = _LMILP->getarcid(C.second, nodefirst);

		assert(edgefirst != edgesecond);

		IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, _LMILP->_x[edgefirst] + _LMILP->_x[edgesecond] - _LMILP->_y[C.first] - _LMILP->_y[C.second], 0, BandC::LogCompCut.c_str());
		
		Cuts.push(R);
		
	}

	Y.end();
	return Cuts;
}

vector<IloRange*> BandCCallBack::collectCutsOnRoutes(CutPool& Cuts, const vector<vector<int>>& Routes, LRSolution* S, double target)
{
	for (auto& p : Routes) {

		assert(S->isinitialdepot(p.front()));
		assert(S->isfinaldepot(p.back()));

		auto corepath(p);
		corepath.erase(corepath.begin());
		corepath.pop_back();

		double xval = S->getXVal(p);
		double xvalcore = S->getXVal(corepath);
		double score = S->score(p);

		auto n = count_if(p.begin(), p.end(), [&](int i) {return _I->Node(i)->isDepot() || _I->Node(i)->isMandatoryNode(); });

		bool cutfound = false;
		if (S->hasConflict(corepath) && corepath.size()>=2 ) { 
			cutfound = Cuts.push(S->genPathCut(corepath));
		}
		
		if (!cutfound && score <= target) {
			//Path Cut (based on score)
			assert(n >= 2);
			cutfound = Cuts.push(S->genCycleCoverCut(p,2));
		}
		
		if (!cutfound && S->length(p) > _I->Tmax()) {
			//Cut based on length
			cutfound = Cuts.push(S->genInfPathCut(p));
			if (p.size() > 3) {
				vector<int> rPath(p);
				reverse(rPath.begin() + 1, rPath.end() - 2);
				if (S->length(rPath) > _I->Tmax())
					cutfound = Cuts.push(S->genInfPathCut(rPath));
			}

		}
		
	}

	
	return Cuts;
}

vector<IloRange*> BandCCallBack::collectCutsOnFlow(CutPool& Cuts, LRSolution* S, double target, double threshold)
{

	vector<bool> isCandidate(_I->nnodes(), true);
	isCandidate[S->initialdepotid()] = false;
	isCandidate[S->finaldepotid()] = false;
	IloNumArray Y = S->getYVal();
	

	vector<int> nodes;
	for (int i = 0; i < _I->nnodes(); ++i) if (Y[i] > threshold && !S->isdepot(i)) nodes.push_back(i);
	sort(nodes.begin(), nodes.end(), [Y](int i, int j) {return Y[i] > Y[j]; });
	Y.end();

	for (auto i : nodes) if (isCandidate[i]) {
		if (_LMILP->timeLeft() < 0) {
			cout << "_LMILP timeLeft: " << _LMILP->timeLeft() << endl;
			break;
		}

		vector<int> Sources;
		vector<int> Targets;

		Sources.push_back(i);
		Targets.push_back(S->initialdepotid());
		Targets.push_back(S->finaldepotid());

		auto P = S->getShoresMinCut(Sources, Targets);
		
		double flow(S->getXVal(P.first, P.second));
		
		bool cutfound = false;

		auto n = count_if(P.first.begin(), P.first.end(), [&](int i) {return _I->Node(i)->isDepot() || _I->Node(i)->isMandatoryNode(); });


		//Conflict Cuts
		if (containConflictPair(P.first)) {
			cutfound = Cuts.push(S->genPathCut(P.first));
		}

		//Subtour elimination
		if (!cutfound && P.first.size() > 1) {
			if (n >= 1) cutfound = Cuts.push(S->genCycleCoverCut(P.first));
			if (!cutfound) cutfound = Cuts.push(S->genCycleCut(P.first));
		}

		//Connectivity cuts
		if (!cutfound) {
			cutfound = Cuts.push(S->genFlowCut(P.first, P.second, i));
		}	

		if (cutfound) for (auto s : P.first) isCandidate[s] = false;

	}
	
	for (auto i : nodes) isCandidate[i] = true;
	for (auto i : nodes) if (isCandidate[i] ) 
	{
		if (_LMILP->timeLeft() < 0) break;

		vector<int> Sources;
		vector<int> Targets;

		Sources.push_back(S->initialdepotid());
		Sources.push_back(S->finaldepotid());
		Targets.push_back(i);

		auto P = S->getShoresMinCut(Sources, Targets);

		bool cutfound = false;

		auto n = count_if(P.first.begin(), P.first.end(), [&](int i) {return _I->Node(i)->isDepot() || _I->Node(i)->isMandatoryNode(); });
		double score = S->score(P.first);

		if(score <= target)
			cutfound = Cuts.push(S->genCycleCoverCut(P.first, 2));

		

		//Conflict Cuts
		if (!cutfound && containConflictPair(P.first)) {
			cutfound = Cuts.push(S->genPathCut(P.first));
		}

		//Subtour elimination
		if (!cutfound && P.first.size() > 1) {
			if (n >= 1) cutfound = Cuts.push(S->genCycleCoverCut(P.first));
			else
				cutfound = Cuts.push(S->genCycleCut(P.first));
		}

		//Connectivity cuts
		if (!cutfound) {
			cutfound = Cuts.push(S->genSECCut(P.first, P.second, i));
		}

		if (cutfound) for (auto s : P.second) isCandidate[s] = false;

	}

	
	return Cuts;
}

vector<IloRange*> BandCCallBack::collectCutsOnConflicts(CutPool& Cuts, LRSolution* S, double target, double threshold)
{
	IloNumArray Y = S->getYVal();
	
	vector< pair<int, int> > conflict;
	copy_if(_ConflictPairs.begin(), _ConflictPairs.end(), back_inserter(conflict), [Y, threshold](pair<int, int> a) {return Y[a.first] > threshold && Y[a.second] > threshold; });

	sort(conflict.begin(), conflict.end(), [Y](pair<int, int> a, pair<int, int> b) {return Y[a.first] + Y[a.second] < Y[b.first] + Y[b.second]; });
	
	double** Network = S->extractNetwork();

	while (!conflict.empty()) {
		auto C = conflict.back();
		conflict.pop_back();

		auto path = S->bfs(Network, S->nnodes(), C.first, C.second);
		

		if (path.empty()) continue;

		bool cutfound = false;

		if (!cutfound) {
			cutfound = Cuts.push(S->genPathCut(path));
		}

	
		if (cutfound) {
			auto it = conflict.begin();
			while (it != conflict.end()) {
				auto iti = find(path.begin(), path.end(), it->first);
				auto itj = find(path.begin(), path.end(), it->second);
				if (iti != path.end() && itj != path.end()) it = conflict.erase(it);
				else it++;
			}
		}

	}

	Y.end();
	for (int i = 0; i < _I->nnodes(); ++i) delete[] Network[i];
	delete[] Network;
	return Cuts;
}

vector<IloRange*> BandCCallBack::collectMatchingCuts(CutPool& Cuts, LRSolution* S)
{
	vector<double> X = S->getSetValues();
	sort(X.begin(), X.end(), [](double x1, double x2) {return x1 > x2; });

	vector<edge> E;
	for (int i = 0; i < _I->nnodes(); ++i) for (int j = 0; j < i; j++) {
		double w = S->getXVal(i, j);
		if (w > EPS ) E.push_back(edge(i, j, w));
	}
	sort(E.begin(), E.end(), [](edge& a, edge& b) {return a.w < b.w; });

	
	bool cutfound = false;
	for (auto threshold : X) if(!cutfound && _LMILP->timeLeft() > 0){
		vector<int> parent(_I->nnodes(), -1);
		vector<int> rank(_I->nnodes(), 1);

		vector<edge> Et;
		copy_if(E.begin(), E.end(), back_inserter(Et), [threshold](edge& e) {return e.w < threshold; });
		sort(Et.begin(), Et.end(), [](edge& a, edge& b) {return a.w < b.w; });

	
		vector<edge> Tree;
		for (auto& currentedge : Et) {
			if (findparent(currentedge.i, parent) != findparent(currentedge.j, parent)) {
				unite(currentedge.i, currentedge.j, parent, rank);
				Tree.push_back(currentedge);

				//We get the comb
				vector<int> H;
				stack<int> Stack;
				Stack.push(currentedge.i);
				vector<bool> visited(_I->nnodes(), false);
				visited[S->initialdepotid()] = visited[S->finaldepotid()] = true;
				while (!Stack.empty()) {
					int i = Stack.top();
					Stack.pop();
					if (visited[i])
						continue;
					H.push_back(i);
					visited[i] = true;
					for (auto& e : Tree)
						if (e.i == i && !visited[e.j]) Stack.push(e.j);
						else if (e.j == i && !visited[e.i]) Stack.push(e.i);

				}

				//We get the arcs leaving H in the current Tree
				vector<bool> isCandidate(_I->nnodes(), true);
				for (auto i : H) isCandidate[i] = false;
				vector< edge > DeltaH;
				copy_if(E.begin(), E.end(), back_inserter(DeltaH), [isCandidate](edge& e) {return isCandidate[e.i] != isCandidate[e.j]; });
				
				if (DeltaH.empty()) continue;

				//We get the Teeth
				sort(DeltaH.begin(), DeltaH.end(), [](edge& a, edge& b) {return a.w < b.w; });
				vector< edge > T;
				T.push_back(DeltaH.back());
				DeltaH.pop_back();
				while (DeltaH.size() >= 2) {
					edge e1 = DeltaH.back(); DeltaH.pop_back();
					edge e2 = DeltaH.back(); DeltaH.pop_back();
					if (e1.w + e2.w - 1 <= EPS) break;
					else {
						T.push_back(e1);
						T.push_back(e2);
					}
				}

				//We repair the teeth
				auto copyT(T);
				unordered_set< int > VT;
				for (auto& e : T) {
					VT.insert(e.i);
					VT.insert(e.j);
				}
				for (auto i : VT) {
					auto n = count_if(T.begin(), T.end(), [i](edge& a) {return a.i == i || a.j == i; });

					if (n >= 2) {
						auto it = find(H.begin(), H.end(), i);
						if (it != H.end()) H.erase(it);
						else H.push_back(i);
						auto last = remove_if(T.begin(), T.end(), [i](edge& a) {return a.i == i || a.j == i; });
						assert(last != T.end());
						T.erase(last, T.end());
					}
				}

				auto c = count_if(H.begin(), H.end(), [currentedge](int i) {return currentedge.i == i || currentedge.j == i; });
				if (c != 2) continue;

				if (T.size() < 1) continue;
				if (T.size() % 2 == 0) T.erase(min_element(T.begin(), T.end(), [](edge& a, edge& b) {return a.w < b.w; }));
				if (T.size() >= 3) {
					vector< pair<int, int> >TE;
					for (auto& e : T) TE.push_back(pair<int, int>(e.i, e.j));
					cutfound= Cuts.push(S->genMatchingCut(H, TE)) || cutfound;

				}


			}
		}
	}

	return Cuts;
}

void BandCCallBack::savelog()
{
	// save statistics in a file
	string outputFile = "LogBandC.txt";

	vector<string> CutType;
	CutType.push_back(BandC::CycleCut);
	CutType.push_back(BandC::PathCut);
	CutType.push_back(BandC::ConflictPathCut);
	CutType.push_back(BandC::CycleCoverCut);
	CutType.push_back(BandC::CycleCoverCut2);
	CutType.push_back(BandC::CondCut);
	CutType.push_back(BandC::LogCut);
	CutType.push_back(BandC::LogCompCut);
	CutType.push_back(BandC::FlowCut);
	CutType.push_back(BandC::InCompCut);
	CutType.push_back(BandC::InfPathCut);
	CutType.push_back(BandC::MatchCut);
	CutType.push_back(BandC::SECCut);
	CutType.push_back(BandC::UnknownCut);



	int w = 15;


	fstream output;
	output.open(outputFile.c_str(), ios::in);
	output.setf(ios::left);
	output.precision(2);
	output.setf(ios::fixed, ios::floatfield);

	if (!output.is_open()) {
		ofstream output(outputFile.c_str());
		output << setw(w) << "Instance" << setw(w) << "Status" << setw(w) << "Threads";
		for (auto& s : CutType) output << setw(w + 5) << s;
		output << endl;
		output.close();
	}
	else output.close();


	output.setf(ios::left);
	output.precision(2);
	output.setf(ios::fixed, ios::floatfield);
	output = fstream(outputFile.c_str(), ios::app);
	string name = _I->instancename();
	size_t lastSeparator = name.find_last_of("/\\");
	if (lastSeparator != std::string::npos)
		name = name.substr(lastSeparator + 1);

	output << setw(w) << name << setw(w) << _LMILP->_cplex.getStatus() << setw(w) << _LMILP->_threads;
	for (auto& s : CutType) {
		int n = 0;
		for (int t = 0; t < _LMILP->_threads; ++t)
			for (auto& L : _Log[t]) if (L.first == s) n += L.second;
		output << setw(w + 5) << n;
	}
	output << endl;

	output.close();
}

void BandCCallBack::addBranches(const IloCplex::Callback::Context& context)
{
	try {
		if (!context.inBranching()) return;

		IloNum z = context.getRelaxationObjective();

		LRSolution* _S = new LRSolution(_LMILP, [&context](IloNumVarArray& X, IloNumArray& V) { context.getRelaxationPoint(X, V); });
		IloNumArray Y = _S->getYVal();

		bool branchcreated = false;

		if (!branchcreated) {
			int ibest = -1;
			double evbest = DBL_MAX;
			double scorebest = -1;
			for (int i = 0; i < _I->nnodes(); ++i) if (!_S->isdepot(i)) {

					double val = Y[i];

					double ev = fabs(0.5 - val);
					if (ev >= 0.5 - EPS) continue;

					double score = fmin(1 - val, val) * _I->Node(i)->score();
					if (score > scorebest + EPS || (fabs(score - scorebest) < EPS && evbest > ev)) {
						ibest = i;
						evbest = ev;
						scorebest = score;
					}
			}
			if (evbest < 0.5 - EPS) {
				assert(ibest >= 0);

				context.makeBranch(_LMILP->_y[ibest], 1, IloCplex::BranchUp, z);
				context.makeBranch(_LMILP->_y[ibest], 0, IloCplex::BranchDown, z);
				branchcreated = true;
			}

		}


		if (!branchcreated) {

			int aidbest = -1;
			double evbest = DBL_MAX;
			double scorebest = 0;
			
			for (int i = 0; i < _I->nnodes(); ++i) for (int j = 0; j < i; j++) {
				arc* A = _LMILP->getArc(i,j);
				int aid = A->id();
				double val = _S->getXVal(aid);
				double ev = fabs(0.5 - val);
				double score = fmin(1.0 - val, val) * (_I->Node(A->geti())->score() + _I->Node(A->getj())->score());
				if (ev < 0.5 - EPS && (score > scorebest + EPS || (fabs(score - scorebest) < EPS && evbest > ev))) {
					aidbest = aid;
					evbest = ev;
					scorebest = score;
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

		Y.end();
	}
	catch (IloException& ex) {
		cout << "Error Branching: " << ex << endl;
	}
}


void BandCCallBack::invoke(const IloCplex::Callback::Context& context)
{

	addLazyCuts(context);

	addUserCuts(context);


	searchSolutionFromLP(context);

}