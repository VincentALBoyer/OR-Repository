#pragma once
#include "Solver.h"

class LRSolution;
class BandCCallBack;

class arc {
	int _i;
	int _j;
	int _edgeid;
public:
	arc(int i, int j, int id) :_edgeid(id) {
		_i = max(i, j);
		_j = min(i, j);
	}
    bool isConnectedTo(int i) const { return _i == i || _j == i; }
	pair<int, int> getVertices() { return pair<int, int>(_i, _j); }
	int id() const { return _edgeid; }
    int geti() const { return _i; }
    int getj() const { return _j; }
};

class BandC :  public Solver
{
    friend class LRSolution;
    friend class BandCCallBack;
protected:
	IloEnv _env;
	IloCplex _cplex;
	IloModel _model;
	IloObjective _obj;

	IloNumVarArray _x;

	IloNumVarArray _y;

	IloRangeArray _Constraints;

	vector<arc*> _Edges;

	void solvemethod(Solution* S);

	int getarcid(int i, int j) {
		if (i == j) return -1;
		int _i = max(i, j);
		int _j = min(i, j);
		return  _i * (_i - 1) / 2 + _j;
	}

	arc* const getArc(int i, int j) {
		if (i == j) return NULL;
		else return _Edges[getarcid(i, j)];
	}

	vector<arc* > getConnectedArc(int i) {
		vector<arc*> A;
        for (int j = 0; j < _I->nnodes(); ++j) if (i != j)
            A.push_back(getArc(i, j));
		return A;
	}

	bool isconnected(int i, int j, int k) { return getArc(i, j) != NULL; }

	IloNumVar const& getXVar(int i, int j, int k) { return _x[getarcid(i, j)]; }

public:

    static const string CycleCut;
    static const string PathCut;
    static const string ConflictPathCut;
    static const string CycleCoverCut;
    static const string CycleCoverCut2;
    static const string CondCut;
    static const string LogCut;
    static const string LogCompCut;
    static const string FlowCut;
    static const string InCompCut;
    static const string InfPathCut;
    static const string MatchCut;
    static const string SECCut;
    static const string UnknownCut;

	BandC(Instance* I);

	~BandC();

	double gap() { 
        double gap = 10;
        try
        {
           gap= _cplex.getMIPRelativeGap();
        }
        catch (const IloException ex)
        {
            cout << ex << endl;
        }
        return gap;
    }

	Solution* recoversolution();

};

class LRSolution {

    Instance* _I;

    double* _XVal;

    BandC* _LMILP;

    std::function<void(IloNumVarArray&, IloNumArray&)> _getValues;

public:

    LRSolution(BandC* M, const std::function<void(IloNumVarArray&, IloNumArray&)>& getValues) : _LMILP(M) {
        _I = _LMILP->_I;

        IloNumArray XVal = IloNumArray(_LMILP->_env);

        _getValues = getValues;
        _getValues(_LMILP->_x, XVal);

        _XVal = new double [XVal.getSize()];
        for (int i = 0; i < XVal.getSize(); ++i) _XVal[i] = XVal[i];
        XVal.end();
    }

    ~LRSolution() {
        delete[] _XVal;
    }

    void setXVal(int i, int j, double val) { _XVal[_LMILP->getarcid(i, j)] = val; }
    double getXVal(int i, int j) {
        if (i == j) return 0.0;
        try {
            return _XVal[_LMILP->getarcid(i, j)];
        }
        catch (IloException& e) {
            cout << e.getMessage() << endl;
            exit(1000);
        }
    }

    double getXVal(int aid) { return _XVal[aid]; }

    double getXVal(const vector<int>& V) {
        double val = 0;
        for (auto u : V) for (auto v : V) if (u < v)
            val += getXVal(u, v);
        return val;
    }


    double getXVal(const vector<int> H, const vector<int> T) {
        double val = 0;
        for (auto u : H) for (auto v : T) if (u != v)
            val += getXVal(u, v);
        return val;
    }

    IloNumArray getYVal() {
        IloNumArray YVal(_LMILP->_env);
        _getValues(_LMILP->_y, YVal);
        return YVal;
    }

    vector<double> getSetValues() {
        vector<double> X;

        IloNumArray Y = getYVal();
        for (int i = 0; i < _I->nnodes(); ++i) if (Y[i] > EPS)
            X.push_back(ceil(Y[i] * 100) / 100.0);
        Y.end();


        sort(X.begin(), X.end());
        auto last = unique(X.begin(), X.end());
        X.erase(last, X.end());
       
        return X;
    }

    double** extractNetwork() {
        double** Network = new double* [_I->nnodes()];
        for (int i = 0; i < _I->nnodes(); ++i) {
            Network[i] = new double[_I->nnodes()];
            Network[i][i] = 0.0;
            for (int j = 0; j < i; ++j) {
                Network[i][j] = Network[j][i] = getXVal(i, j);
            }
        }
        return Network;
    }

    BandC* getMILP() { return _LMILP; }
    Instance* instance() { return _I; }
    bool isdepot(int i) { return _I->Node(i)->isDepot(); }
    int initialdepotid() { return _I->InitialDepot()->id(); }
    int finaldepotid() { return _I->FinalDepot()->id(); }
    int nnodes() { return _I->nnodes(); }


    bool isinitialdepot(int i) { return _I->Node(i)->isInitialDepot(); }
    bool isfinaldepot(int i) { return _I->Node(i)->isFinalDepot(); }

    void reset() {
        for (int l = 0; l < _LMILP->_x.getSize(); ++l) _XVal[l] = 0.0;
    }

    double fitness(vector<int>& R) { 
        if (!isFeasible(R)) return 0.0;
        return this->score(R); 
    }

    bool isFeasible(vector<int>& R) {
        for (auto i : R) assert(_I->Node(i)->isFeasible());
        if(!isinitialdepot(R.front()) || !isfinaldepot(R.back())) return false;
        if (length(R) > _I->Tmax()) return false;
        vector<bool> M(nnodes(), false);
        for (auto i : R) M[i] = true;
        for (int i = 0; i < nnodes(); ++i) if (_I->Node(i)->isMandatoryNode() && !M[i]) return false;

        return true;
    }


    IloRange* genCycleCut(const vector<int>& NodeSet) {
        assert(!NodeSet.empty());

        IloExpr expr(_LMILP->_env);

        for (auto i : NodeSet) for (auto j : NodeSet) if (j < i)
                expr += _LMILP->_x[_LMILP->getarcid(i, j)];

        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, (int)NodeSet.size() - 1,BandC::CycleCut.c_str());

        expr.end();
        return R;

    }

    IloRange* genPathCut(const vector<int>& NodeSet) {
        assert(!NodeSet.empty());

        IloExpr expr(_LMILP->_env);
        for (auto i : NodeSet) for (auto j : NodeSet) if (j < i)
            expr += _LMILP->_x[_LMILP->getarcid(i, j)];

        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, (int)NodeSet.size() - 2, BandC::PathCut.c_str());

        expr.end();
        return R;

    }

    IloRange* genConflictPathCut(const vector<int>& NodeSet, int u, int v) {
        assert(!NodeSet.empty());
        assert(!_I->areCompatible(_I->Node(u), _I->Node(v)));

        IloExpr expr(_LMILP->_env);
        for (auto i : NodeSet) for (auto j : NodeSet) if (j < i)
            expr += _LMILP->_x[_LMILP->getarcid(i, j)];

        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr-_LMILP->_y[u]- _LMILP->_y[v], (int)NodeSet.size(), BandC::ConflictPathCut.c_str());

        expr.end();
        return R;

    }

    IloRange* genCycleCoverCut(const vector<int>& NodeSet,  int d=1) {
       
        assert(!NodeSet.empty());

        auto n = count_if(NodeSet.begin(), NodeSet.end(), [&](int i) {return _I->Node(i)->isDepot() || _I->Node(i)->isMandatoryNode(); });
        assert(n >= d);

        IloExpr expr(_LMILP->_env);
        for (auto i : NodeSet) expr -= _LMILP->_y[i];
        for (auto i : NodeSet) for (auto j : NodeSet) if (j < i)
            expr += _LMILP->_x[_LMILP->getarcid(i, j)];

        string name = BandC::CycleCoverCut;
        if (d == 2) name = BandC::CycleCoverCut2;
        else if (d > 2 || d<=0) name = BandC::UnknownCut;
        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, -d, name.c_str());

        expr.end();
        return R;

    }

    IloRange* genConditionalCut(const vector<int>& NodeSet) {
        
        assert(!NodeSet.empty());
        assert(find(NodeSet.begin(), NodeSet.end(), initialdepotid()) != NodeSet.end());
        assert(find(NodeSet.begin(), NodeSet.end(), finaldepotid()) != NodeSet.end());


        IloExpr expr(_LMILP->_env);
        for (auto i : NodeSet) if (!isdepot(i)) expr -= _LMILP->_y[i];
        for (auto i : NodeSet) for (auto j : NodeSet) if (j < i && getXVal(i, j)>EPS)
            expr += _LMILP->_x[_LMILP->getarcid(i, j)];

        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, 1, BandC::CondCut.c_str());

        expr.end();
        return R;
    }

    IloRange* genLogicalConstraint(int i, int j) {
        

        assert(!isdepot(i));
        assert(i != j);
        IloExpr expr(_LMILP->_env);

        expr += _LMILP->_x[_LMILP->getarcid(i, j)];
        expr -= _LMILP->_y[i];
        
        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, 0, BandC::LogCut.c_str());

        expr.end();
        return R;
    }

    //Note. also refered as generalized subtour elimination constraints (GSECs)
    IloRange* genSECCut(const vector<int>& Shore1, const vector<int>& Shore2, int s) {
        
        assert(!Shore1.empty() && !Shore2.empty());
        int degree = 2;
        assert (find(Shore1.begin(), Shore1.end(), initialdepotid()) != Shore1.end());
        assert(find(Shore1.begin(), Shore1.end(), finaldepotid()) != Shore1.end());

        IloExpr expr(_LMILP->_env);
        
        expr -= degree * _LMILP->_y[s];
        for (auto i : Shore1) for (auto j : Shore2)
            expr += _LMILP->_x[_LMILP->getarcid(i, j)];        

        IloRange* R = new IloRange(_LMILP->_env, 0, expr, IloInfinity, BandC::SECCut.c_str());

        expr.end();
        return R;
    }

    IloRange* genFlowCut(const vector<int>& Shore1, const vector<int>& Shore2, int s) {

        assert(!Shore1.empty() && !Shore2.empty());
        int degree = 2;
        if (find(Shore1.begin(), Shore1.end(), initialdepotid()) != Shore1.end()) degree--;
        if (find(Shore1.begin(), Shore1.end(), finaldepotid()) != Shore1.end()) degree--;
        assert(degree > 0);
        IloExpr expr(_LMILP->_env);

        expr -= degree * _LMILP->_y[s];
        for (auto i : Shore1) for (auto j : Shore2)
            expr += _LMILP->_x[_LMILP->getarcid(i, j)];


        IloRange* R = new IloRange(_LMILP->_env, 0, expr, IloInfinity, BandC::FlowCut.c_str());

        expr.end();
        return R;
    }

    IloRange* genIncompatibilityCut(const vector<int>& Shore1, const vector<int>& Shore2, int s1, int s2) {
        
        assert(!Shore1.empty() && !Shore2.empty());
        int degree = 2;
        if (find(Shore1.begin(), Shore1.end(), initialdepotid()) != Shore1.end()) degree--;
        if (find(Shore1.begin(), Shore1.end(), finaldepotid()) != Shore1.end()) degree--;
        assert(degree > 0);
        IloExpr expr(_LMILP->_env);
        
        expr -= degree * (_LMILP->_y[s1] + _LMILP->_y[s2]);
        for (auto i : Shore1) for (auto j : Shore2)
            expr += _LMILP->_x[_LMILP->getarcid(i, j)];
        

        IloRange* R = new IloRange(_LMILP->_env, 0, expr, IloInfinity, BandC::InCompCut.c_str());

        expr.end();
        return R;
    }

    IloRange* genInfPathCut(const vector<int>& NodeSet) {
       
        assert(!NodeSet.empty());


        IloExpr expr(_LMILP->_env);
        for (auto iti = NodeSet.begin(); iti != NodeSet.end(); ++iti) if (!isdepot(*iti)) {
            int i = *iti;
            for (auto itj = iti + 1; itj != NodeSet.end(); ++itj) if (!isdepot(*itj)) {
                int j = *itj;
                expr += _LMILP->_x[_LMILP->getarcid(i, j)];
                break;
            }
        }

        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, (int)NodeSet.size() - 2, BandC::InfPathCut.c_str());

        expr.end();
        return R;
    }


    IloRange* genMatchingCut(const vector<int>& H, const vector< pair<int, int> >& T) {
       
        assert(!H.empty() && !T.empty());
        assert(T.size() % 2 == 1);

        int n = 0;

        IloExpr expr(_LMILP->_env);
       
        for (auto i : H) expr -= _LMILP->_y[i];
        for (auto& a : T)
            expr += _LMILP->_x[_LMILP->getarcid(a.first, a.second)];
        for (auto i : H) for (auto j : H) if (j < i)
            expr += _LMILP->_x[_LMILP->getarcid(i, j)];
        

        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, (T.size() - 1) / 2.0 + n, BandC::MatchCut.c_str());

        expr.end();
        return R;
    }

    int score(int i) {
        if (i >= 1 && i < _I->nnodes() - 1) return _I->Node(i)->score();
        else return 0;
    }


    bool isInteger(int i, int j) {
        double val = getXVal(i, j);
        return val <= EPS || val >= 1.0 - EPS;
    }

    double score(const vector<int>& R) {
        return accumulate(R.begin(), R.end(), 0.0, [&](double a, int i) {
            return a + _I->Node(i)->score();
            });
    }

    double length(const vector<int>& R) {

        double L = 0.0;
        int i = R.front();
    
        for (auto j : R) if (i != j) {

            L += _I->travellingtime(_I->Node(i), _I->Node(j));

            i = j;
        }
        return L;
    }

    vector<int> getRandomPath(int s = -1, double threshold = EPS);

    vector<int> getRandomInfPath(int s);

    vector<int> getHeuristicRoute();

    void extendRandomPath(vector<int>& Path, double threshold = EPS) {
        vector<bool> _RStatus(nnodes(), false);
        for (auto i : Path) _RStatus[i] = true;

        do {
            int j = Path.back();
            _RStatus[j] = true;

            vector<int> Candidates;
            for (int i = 0; i < nnodes(); ++i) if (!_RStatus[i] && getXVal(i, j) > threshold)
                Candidates.push_back(i);


            int i = finaldepotid();
            if (!Candidates.empty()) {
                i = *RouletteWheelSelection(Candidates.begin(), Candidates.end(), [&](int i) {return  round(getXVal(i, j) * 100);  });
                Path.push_back(i);
            }
            else {
                break;
            }



        } while (!isdepot(Path.back()));
       
    }

    bool hasArc(int i, int j) { return getXVal(i, j) > EPS; }

    bool hasConflict(vector<int>& V) {
        for (auto u : V) for (auto v : V) if (u < v) {
            if (!_I->areCompatible(_I->Node(u),_I->Node(v))) return true;
        }
        return false;
    }

    vector< pair<int, int> > getConflictNodes(vector<int>& V) {
        vector< pair<int, int> > C;
        for (auto u : V) for (auto v : V) if (u < v) {            
            if (!_I->areCompatible(_I->Node(u), _I->Node(v))) C.push_back(pair<int, int>(u, v));
        }

        return C;
    }

    vector<int> bfs(double** RNetwork, int n, int s, int t, double threshold = EPS);

    vector<int> bfs(double** RNetwork, int n, vector<int>& S, vector<int>& T, double threshold = EPS);

    vector<int> strongbfs(double** RNetwork, int n, vector<int>& S, vector<int>& T, double threshold = EPS);


    pair< vector<int>, vector<int> > getShoresMinCut(int s, int t = -1);

    pair< vector<int>, vector<int> > getShoresMinCut(vector<int>& S, vector<int>& T);

    vector<int> getRoute();

    void repairroute(vector<int>& R, double MinScore = 0.0);

    void adjustRoutesLength(vector<int>& R);

    void optimizeRoutesLength(vector<int>& R);

    void improveRoutes(vector<int>& R);

    bool bestinsert(vector<int>& R, int s) {
        double l = length(R);
        auto bestit = R.end();
        double bestdelta = DBL_MAX;
        int i = R.front();
        for (auto it = R.begin(); it != R.end(); ++it) if (i != *it) {
            assert(!isfinaldepot(i));
            int j = *it;
            double dij = _I->travellingtime(_I->Node(i), _I->Node(j));
            double dis = _I->travellingtime(_I->Node(i), _I->Node(s));
            double dsj = _I->travellingtime(_I->Node(s), _I->Node(j));
            double delta = dis + dsj - dij;

            if (delta < bestdelta && l + delta <= _I->Tmax()) {
                bestdelta = delta;
                bestit = it;
            }

            i = j;
        }

        if (bestit != R.end()) {
            bestit = R.insert(bestit, s);
        }

        return bestit != R.end();
    }

    int swapinsert(vector<int>& R, int s) {
        double F = fitness(R);
        vector<int> Candidate;
        copy_if(R.begin(), R.end(), back_inserter(Candidate), [&](int i) { return !isdepot(i) && !_I->Node(i)->isMandatoryNode() && _I->Node(i)->score() < _I->Node(s)->score(); });

        if (Candidate.empty()) return s;

        sort(Candidate.begin(), Candidate.end(), [&](int i, int j) {return _I->Node(i)->score() < _I->Node(j)->score(); });

        for (auto i : Candidate) {
            auto it = find(R.begin(), R.end(), i);
            assert(it != R.end());
            *it = s;
            double l = length(R);
            double newF = fitness(R);
            if (l <= _I->Tmax() && newF > F)
                return i;
            else *it = i;
        }

        return s;
    }

    pair< IloNumVarArray*, IloNumArray* > extractSolution();

	void exportSubGraph(const string& filename, vector<int>& S, int level=0);
};

class CutPool : public vector< IloRange* > {

    std::function<IloNum(IloExpr&)> _evalExpr ;

    double _mindisttocut;

public:
    CutPool(const std::function<IloNum(IloExpr&)>& eval, double mindist = 0.01) : vector< IloRange* >(), _evalExpr(eval), _mindisttocut(mindist) {}
    ~CutPool() {}

    bool isvalidcut(IloRange* R) {
        if (R == NULL) return false;
        IloExpr expr = R->getExpr();
        IloNum val = _evalExpr(expr);
        IloNum lhs = R->getLB();
        IloNum rhs = R->getUB();

        IloNum threshold(EPS);

        expr.end();
        return lhs - val > threshold || val - rhs > threshold;
    }

    double DistanceToCut(IloRange* R) {
        if (R == NULL) return 0.0;
        IloExpr expr = R->getExpr();      
        IloNum lhs = R->getLB();
        IloNum rhs = R->getUB();

        IloExpr::LinearIterator it = expr.getLinearIterator();
        double qsum = 0.0;
        while (it.ok()) {
            if(it.getVar().getUB()>EPS)
                qsum += pow(it.getCoef(), 2);
            ++it;
        }
        if (qsum < 1.0 - EPS) {
            cout << *R << endl;
        }
        if (qsum < EPS) {
            expr.end();
            return NULL;
        }
        assert(qsum >= 1.0);
        qsum = sqrt(qsum);

        IloNum val = _evalExpr(expr);
        double diff = fmax((lhs - val), (val - rhs));

        double dist = diff / qsum;
        
        string name = R->getName();
        if (diff >= 0.001 && name == "LogCut")
   
        assert(fabs(diff) >= fabs(dist));

        expr.end();
        return dist;
    }

    bool push(IloRange* R) {
        if (R == NULL) return false;
        double dist = DistanceToCut(R);
        bool add = (dist > _mindisttocut);

        if (add)  push_back(R);
        else {
            R->end();
            delete R;
        }
        return add;
    }



    void clean() {
        while (!empty()) {
            auto Cut = back();
            pop_back();
            Cut->end();
            delete Cut;
        }
    }
};


class BandCCallBack : public IloCplex::Callback::Function {
private:

    struct edge {
        int i;
        int j;
        double w;

        edge(int s, int t, double x) : i(s), j(t), w(x) {};
    };

    Instance* _I;

    BandC* _LMILP;

    bool _enablelog;
    vector < pair<string, int> >* _Log;

    vector< pair<int, int> > _ConflictPairs;

    bool containConflictPair(const vector<int>& S) {
        if (S.size() <= 1) return false;
        vector<bool> Status(_I->nnodes(), false);
        for (auto i : S) Status[i] = true;
        auto it = find_if(_ConflictPairs.begin(), _ConflictPairs.end(), [Status](pair<int, int> C) {return Status[C.first] && Status[C.second]; });
        return it != _ConflictPairs.end();
    }

    void addLazyCuts(const IloCplex::Callback::Context& context);

    void addUserCuts(const IloCplex::Callback::Context& context);

    void searchSolutionFromLP(const IloCplex::Callback::Context& context);

    bool addSolution(const IloCplex::Callback::Context& context, LRSolution* S);

    bool isvalidcut(IloRange* R, const std::function<IloNum(IloExpr&)>& getValue);

    bool hasDuplicates(const std::vector< std::pair<int, int> >& pairs) {

        for (auto& p : pairs) {
            auto n = count_if(pairs.begin(), pairs.end(), [p](pair<int, int> a) {
                return (a.first == p.first && a.second == p.second) || (a.second == p.first && a.first == p.second); });
            assert(n == 1);
            if (n > 1)
                return false;
        }

        // No duplicates found
        return false;
    }

    vector<IloRange* > collectLogicalCuts(CutPool& Cuts, LRSolution* S);

    vector<IloRange* > collectCutsOnRoutes(CutPool& Cuts, const vector< vector<int> >& Routes, LRSolution* S, double target);

    vector<IloRange* > collectCutsOnFlow(CutPool& Cuts, LRSolution* S, double target, double threshold = EPS);

    vector<IloRange* > collectCutsOnConflicts(CutPool& Cuts, LRSolution* S, double target, double threshold = EPS);

    void PrintCut(IloRange* Cut, const IloCplex::Callback::Context& context) {
        if (!isvalidcut(Cut, [&](IloExpr& expr) { return context.getCandidateValue(expr); })) {
            double val = context.getCandidateValue(Cut->getExpr());
            double lhs = Cut->getLB();
            double rhs = Cut->getUB();
            cout << ": " << lhs << "<=" << val << "<=" << rhs << endl;
            cout << *Cut << endl;
        }
    }

    double getProb(const IloCplex::Callback::Context& context) {
        double Delta = 0.1;
        double P = 0.01;
        double T = -Delta / log(P);
        double bb = context.getDoubleInfo(IloCplex::Callback::Context::Info::BestBound);
        double f = context.getIncumbentObjective();
        double gap = fabs(bb - f) / bb;
        double r = exp(-(1.0 - gap) / T);
        return r;
    }


    //The Matching Constraints
    int findparent(int i, vector<int>& parent) {
        if (parent[i] == -1) return i;
        else return parent[i] = findparent(parent[i], parent);
    }
    void unite(int i, int j, vector<int>& parent, vector<int>& rank) {
        int s1 = findparent(i, parent);
        int s2 = findparent(j, parent);
        if (s1 != s2) {
            if (rank[s1] < rank[s2])  parent[s1] = s2;
            else if (rank[s1] > rank[s2]) parent[s2] = s1;
            else {
                parent[s2] = s1;
                rank[s1] += 1;
            }
        }
    }
    vector<IloRange* > collectMatchingCuts(CutPool& Cuts, LRSolution* S);

    void updatelog(const IloCplex::Callback::Context& context, CutPool& Cuts) {
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

    void addBranches(const IloCplex::Callback::Context& context);

public:


    // Constructor
    BandCCallBack(BandC* LMILP) : _LMILP(LMILP), _I(LMILP->_I) {

        for (int i = 0; i < _I->nnodes(); ++i) for (int j = i + 1; j < _I->nnodes(); ++j) {
            if (!_I->areCompatible(_I->Node(i),_I->Node(j))) {
                assert(!_I->Node(i)->isDepot() && !_I->Node(j)->isDepot());
                _ConflictPairs.push_back(pair<int, int>(i, j));
            }
        }
        _enablelog = true;
        _Log = new vector < pair<string, int> >[_LMILP->_threads];
    }

    ~BandCCallBack() {
        
        delete[] _Log;
    }

    void printlog() {
        if (!_enablelog) cout << "Log was disabled" << endl;
        else {
            for (int t = 0; t < _LMILP->_threads; ++t) {
                cout << "Thread " << t << ": ";
                for (auto& l : _Log[t]) if (l.second > 0)
                   cout << "[" << l.first << ": " << l.second << "] ";
                if (_Log[t].empty()) cout << "No Cuts";
                cout << endl;
            }
        }
    }

    void savelog();

    // The main callback function
    void invoke(const IloCplex::Callback::Context& context);

};