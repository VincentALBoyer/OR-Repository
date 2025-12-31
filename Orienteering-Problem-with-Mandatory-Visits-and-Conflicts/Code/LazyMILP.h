#pragma once
#include "Solver.h"
#include <queue>

// MACROS to enable/disable some features
#define USE_GSEC true       //yes
#define USE_MC true         //yes
#define USE_CC true         //yes
#define USE_CCC true
#define USE_ICC true        //yes
#define USE_IPC true
#define USE_IPCC true       //yes
#define USE_LC true         //yes

// MACROS for algorithm parameters
#define STOP_AT_ROOT false
#define USE_HEURISTIC true
#define SAVE_ROOT_GRAPH false
#define USE_CUSTOM_BRANCHING false
#define PRELOAD_LC true

class MyCallback;
class CplexSolution;

class arc {
    int _i;
    int _j;
    int _k;
    int _aid;
public:
    arc(int i, int j, int k, int aid) :_aid(aid), _k(k) {
        _i = max(i, j);
        _j = min(i, j);
    }
    bool isConnectedTo(int i) const { return _i == i || _j == i; }
    pair<int, int> getVertices() { return pair<int, int>(_i, _j); }
    int getvid() const { return _k; }
    int arcid() const { return _aid; }
    int geti() const { return _i; }
    int getj() const { return _j; }
};

class LazyMILP : public Solver
{
    friend class MyCallback;
    friend class CplexSolution;

protected:

    double _M;

    int _OF;

    IloEnv _env;                                     //Environnement Cplex
    IloCplex _cplex;
    IloModel _model;
    IloObjective _obj;                               //objective

    IloNumVarArray _x;       //sequence variables i/j
    IloArray<IloNumVarArray > _y;

    IloNumVar _Sm;

    double _mu;

    int _nconstraints;                               //total number of constrants
    IloRangeArray _Constraints;                      //All the constraints
    
    int _narcs;
    arc** _Arcs;  
    
    void solvemethod(Solution* S);
    void initsolver(Solution* S = NULL);
    void initmodel();

    bool isdepot(int nodeid) { return nodeid >= _I->nnodes() - 1 || nodeid == 0; }
    int initialdepotid() { return 0; }
    int finaldepotid() { return _I->nnodes() - 1; }
    int nnodes() { return _I->nnodes(); }

    //int getDestinationId(int vid) { return _I->nnodes() - 1 + vid; }
    bool isinitialdepot(int i) { return i == 0; }
    bool isfinaldepot(int i) { return i >= _I->nnodes() - 1; }


    int getarcid(int i, int j, int k){
        if (i == j) return -1;
        int _i = max(i, j);
        int _j = min(i, j);
        return k * (_I->nnodes() - 1) * (_I->nnodes())/2 + _i * (_i - 1) / 2 + _j;
    }

    arc* const getArc(int i, int j, int k) {
        if (i == j) return NULL;
        else return _Arcs[getarcid(i,j,k)];
    }

    vector<arc* >& getConnectedArc(int i, int k) {
        vector<arc*> A;
        for (int j = 0; j < _I->nnodes(); ++j) if (i != j) A.push_back(getArc(i, j, k));
        return A;
    }

    bool isconnected(int i, int j, int k) { return getArc(i, j, k) != NULL; }

    IloNumVar const & getXVar(int i, int j, int k) { return _x[getarcid(i, j, k)]; }

public:

    static const string CycleCut;
    static const string PathCut;
    static const string SubCycleCut;
    static const string CycleCoverCut;
    static const string PathCoverCut;
    static const string CondCut;
    static const string LogCut;
    static const string FlowCut;
    static const string InCompCut;
    static const string InfPathCut;
    static const string MatchCut;
    static const string ScoreCut;
    static const string SECCut;
    static const string EdgeCut;
    static const string ConnectCut;
    static const string TmaxCut;

    LazyMILP(Instance* I, int OF);

    ~LazyMILP();

    

    double gap() { return _cplex.getMIPRelativeGap(); }
    double value() { return _cplex.getObjValue(); }


    Solution* recoversolution();
};

class CplexSolution {

 
    const double _eps = 0.001;
    int _nnodes;
    Instance* _I;

    IloNumArray _XVal;
    
    LazyMILP* _LMILP;

    std::function<void(IloNumVarArray&, IloNumArray&)> _getValues;

public:
    
    CplexSolution(CplexSolution *S) : _LMILP(S->_LMILP) {
        _I = _LMILP->_I;
        _nnodes = _I->nnodes();
        _getValues = S->_getValues;
        _XVal = S->_XVal;

    }

    CplexSolution(LazyMILP* M, const std::function<void(IloNumVarArray&, IloNumArray&)>& getValues) : _LMILP(M) {
        _I = _LMILP->_I;
        _nnodes = _I->nnodes();
       
        _XVal = IloNumArray(_LMILP->_env);
        
        _getValues = getValues;
        _getValues(_LMILP->_x, _XVal);

    }
    
    ~CplexSolution() {
        _XVal.end();
    }

    IloNumArray getYVal(int k) {
        IloNumArray YVal(_LMILP->_env);
        IloNumVarArray Y(_LMILP->_env);
        for (int i = 0; i < _I->nnodes(); ++i) Y.add(_LMILP->_y[i][k]);
        _getValues(Y, YVal);
        Y.end();
        return YVal;
    }

    IloNum getYVal(int i, int k) {
        IloNumArray YVal(_LMILP->_env);
        IloNumVarArray Y(_LMILP->_env);
        Y.add(_LMILP->_y[i][k]);
        _getValues(Y, YVal);
        Y.end();
        return YVal[0];
    }

    vector<double> getSetValues(int k) {
        vector<double> X;
        for (int i = 0; i < _I->nnodes(); ++i) for (int j = 0; j < i; ++j)
            if(getXVal(i, j, k)>EPS)
                X.push_back(ceil(getXVal(i, j, k) * 100) / 100.0);

        //IloNumArray Y = getYVal(k);
        //for (int i = 0; i < _I->nnodes(); ++i) if(Y[i]>EPS)
        //    X.push_back(ceil(Y[i] * 100) / 100.0);
        //Y.end();


        sort(X.begin(), X.end());
        auto last = unique(X.begin(), X.end());
        X.erase(last, X.end());
       
        return X;
    }

    double** extractVehicleNetwork(int k) {
        double** Network = new double* [_I->nnodes()];
        IloNumArray Y = getYVal(k);
        for (int i = 0; i < _I->nnodes(); ++i) {
            Network[i] = new double[_I->nnodes()];
            Network[i][i] = 0;
            for (int j = 0; j < i; ++j) {
                if (Y[i] < EPS || Y[j] < EPS) Network[i][j] = Network[j][i] = 0.0;
                else Network[i][j] = Network[j][i] = getXVal(i, j, k);
            }
        }
        Y.end();
        return Network;
    }

    double** extractCompleteNetwork() {
        double** Network = new double* [_I->nnodes()];
        for (int i = 0; i < _I->nnodes(); ++i) {
            Network[i] = new double[_I->nnodes()];
            Network[i][i] = 0.0;
            for (int j = 0; j < i; ++j) {
                double w = 0.0;
                for(int k=0;k<_I->nvehicles() && w<1.0;++k) w += getXVal(i, j, k);
                Network[i][j] = Network[j][i] = w;
            }
        }
        return Network;
    }

    double getMinDistance(const vector<int> Nodes) {
        double d = DBL_MAX;
        for (auto i : Nodes) for (auto j : Nodes) if (i < j) {
            d = fmin(d, _I->distance(_I->node(i), _I->node(j)));
        }
        return d;
    }

    double getMaxDistance(const vector<int> Nodes) {
        double d = 0;
        for (auto i : Nodes) for (auto j : Nodes) if (i < j) {
            d = fmax(d, _I->distance(_I->node(i), _I->node(j)));
        }
        return d;
    }

    void pasteVehicleNetwork(double** Network, int k) {
        for (int i = 0; i < _nnodes; ++i) {
            for (int j = 0; j < _nnodes; ++j) Network[i][j] = getXVal(i, j, k);
        }
    }

    LazyMILP* getLazyMILP() { return _LMILP; }
    double getmu() { return _LMILP->_mu; }
    int getOF() { return _LMILP->_OF; }
    Instance* instance() { return _I; }
    bool isdepot(int nodeid) { return nodeid >= _I->nnodes() - 1 || nodeid == 0; }
    int initialdepotid() { return 0; }
    int finaldepotid() { return _I->nnodes() - 1; }
    int nnodes() { return _I->nnodes() ; }


    bool isinitialdepot(int i) { return i == 0; }
    bool isfinaldepot(int i) { return i >= _I->nnodes() - 1; }

    void reset() {
        for (int l = 0; l < _XVal.getSize(); ++l) _XVal[l] = 0.0;
    }

    double fitness();

    double fitness(vector<int>& R);

    double fitness(vector< vector<int> >& Routes);


    template < class CBClass > bool isvalidineq(CBClass* CB, IloRange* R) {

        bool isvalid = false;
        try {
            IloExpr expr = R->getExpr();
            double val = CB->getValue(expr); expr.end();
            double lhs = R->getLB();
            double rhs = R->getLB();

            isvalid = (val + EPS < lhs || val - EPS > rhs );
            expr.end();
        }
        catch (IloException& e) {
            cout << e << endl;
            exit(1000);
        }

        return isvalid;
    }


    IloRange* genCycleCut( const vector<int>& NodeSet, int ks=-1) {
        assert(!NodeSet.empty());
        assert(NodeSet.size() > 1);

        vector<int> K;
        if (ks < 0)  for (int k = 0; k < _I->nvehicles(); ++k) K.push_back(k);
        else K.push_back(ks);
     
        
        IloExpr expr(_LMILP->_env);
        for (auto i : NodeSet) for (auto j : NodeSet) if (j < i) 
            for (int k : K) 
                expr += _LMILP->_x[_LMILP->getarcid(i,j,k)];
        
        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, (int)NodeSet.size() - 1,LazyMILP::CycleCut.c_str());
  
        expr.end();
        return R;


    }

    IloRange* genPathCut(const vector<int>& NodeSet, int ks = -1) {
        assert(!NodeSet.empty());

        vector<int> K;
        if (ks < 0)  for (int k = 0; k < _I->nvehicles(); ++k) K.push_back(k);
        else K.push_back(ks);

        IloExpr expr(_LMILP->_env);
        for (auto i : NodeSet) for (auto j : NodeSet) if (j < i)
            for (int k : K) 
                expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];

        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, (int)NodeSet.size() - 2,LazyMILP::PathCut.c_str());
        //IloRange R(_LMILP->_env, -IloInfinity, expr, (int)NodeSet.size() - 1);

        expr.end();
        return R;

    }

    IloRange* genCycleCoverCut(const vector<int>& NodeSet, int ks=-1) {
        assert(!NodeSet.empty());
        assert(find(NodeSet.begin(), NodeSet.end(), initialdepotid()) != NodeSet.end());
        assert(find(NodeSet.begin(), NodeSet.end(), finaldepotid()) != NodeSet.end());

        vector<int> K;
        if (ks < 0)  for (int k = 0; k < _I->nvehicles(); ++k) K.push_back(k);
        else K.push_back(ks);

        IloExpr expr(_LMILP->_env);
        for (auto i : NodeSet) /*if(!isdepot(i))*/ for (int k : K) expr -= _LMILP->_y[i][k];
        for (auto i : NodeSet) for (auto j : NodeSet) if (j < i) 
            for (int k: K) 
                expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];

        double rhs = -(int)K.size();
        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr,  rhs, LazyMILP::CycleCoverCut.c_str());
        //IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr,  -2);
        //Note: modded by adding +2 for the depots

        expr.end();
        return R;
    }

    IloRange* genPathCoverCut(const vector<int>& NodeSet, int ks = -1) {
        assert(!NodeSet.empty());
        assert(find(NodeSet.begin(), NodeSet.end(), initialdepotid()) != NodeSet.end());
        assert(find(NodeSet.begin(), NodeSet.end(), finaldepotid()) != NodeSet.end());

        vector<int> K;
        if (ks < 0)  for (int k = 0; k < _I->nvehicles(); ++k) K.push_back(k);
        else K.push_back(ks);

        IloExpr expr(_LMILP->_env);
        for (auto i : NodeSet) /*if(!isdepot(i))*/ for (int k : K) expr -= _LMILP->_y[i][k];
        for (auto i : NodeSet) for (auto j : NodeSet) if (j < i)
            for (int k : K)
                expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];

        IloNum rhs(- 2 * (int)K.size());
        
        IloRange * R = new IloRange(_LMILP->_env, -IloInfinity, expr, rhs, LazyMILP::PathCoverCut.c_str());
        

        expr.end();
        return R;

    }

    IloRange* genEdgeCoverCut(const vector<int>& NodeSet, int ks = -1) {
        assert(!NodeSet.empty());
        //assert(find(NodeSet.begin(), NodeSet.end(), initialdepotid()) != NodeSet.end());
        //assert(find(NodeSet.begin(), NodeSet.end(), finaldepotid()) != NodeSet.end());

        vector<int> K;
        if (ks < 0)  for (int k = 0; k < _I->nvehicles(); ++k) K.push_back(k);
        else K.push_back(ks);

        IloExpr expr(_LMILP->_env);
        for (auto i : NodeSet) if (!isdepot(i)) for (int k : K) expr += _LMILP->_y[i][k];
       

        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, IloNum(NodeSet.size() - 1), LazyMILP::EdgeCut.c_str());
        //IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr,  -2);
        //Note: modded by adding +2 for the depots

        expr.end();
        return R;

    }

    IloRange* genConditionalCut(const vector<int>& NodeSet, int k) {
        assert(!NodeSet.empty());
        assert(find(NodeSet.begin(), NodeSet.end(), initialdepotid()) != NodeSet.end());
        assert(find(NodeSet.begin(), NodeSet.end(), finaldepotid()) != NodeSet.end());

       

        IloExpr expr(_LMILP->_env);
        for (auto i : NodeSet) if(!isdepot(i)) expr -= _LMILP->_y[i][k];
        for (auto i : NodeSet) for (auto j : NodeSet) if (j < i && getXVal(i,j,k)>EPS)
            expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];

        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, 1, LazyMILP::CondCut.c_str());
        //IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, -1, name.c_str());
        //Note: modded by adding +2 for the depots

        expr.end();
        return R;

    }

    IloRange* genLogicalConstraint(int i, int j) {
        assert(!isdepot(i));
        assert(i != j);
        IloExpr expr(_LMILP->_env);
       
        for (int k = 0; k < _I->nvehicles(); ++k){
            expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];
            expr -= _LMILP->_y[i][k];
        }
        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, 0, LazyMILP::LogCut.c_str());

        expr.end();
        return R;
    }

    IloRange* genLogicalConstraint(int i, int j, int k) {
        assert(!isdepot(i));
        assert(i != j);
        
        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, _LMILP->_x[_LMILP->getarcid(i, j, k)] - _LMILP->_y[i][k], 0, LazyMILP::LogCut.c_str());

        return R;
    }

    IloRange* genMaxScoreCut(double bestbound) {
        
        IloRange* R = NULL;
        double target = bestbound;
        if (getOF() == 1) 
        {
            target = floor(bestbound);
            R= new IloRange(_LMILP->_env, -IloInfinity, _LMILP->_Sm, target, LazyMILP::ScoreCut.c_str());
        }
        //else if (getOF() == 2) {
        //    double mu = getmu();
        //    double delta = ceil(mu) - mu;
        //    if (bestbound < mu)  double delta = mu - floor(mu);
        //    target = ceil(bestbound - delta) + delta;
        //    R = new IloRange(_LMILP->_env, target, _LMILP->_Sm, IloInfinity, LazyMILP::ScoreCut.c_str());
        //}


        return R;
    }

    //Note. also refered as generalized subtour elimination constraints (GSECs)
    IloRange* genFlowCut(const vector<int>& Shore1, const vector<int>& Shore2, int s, int ks=-1) {
        assert(!Shore1.empty() && !Shore2.empty());

        vector<int> K;
        if (ks < 0)  for (int k = 0; k < _I->nvehicles(); ++k) K.push_back(k);
        else K.push_back(ks);
        int degree=2;
        if (find(Shore1.begin(), Shore1.end(), initialdepotid()) != Shore1.end()) degree--;
        if (find(Shore1.begin(), Shore1.end(), finaldepotid()) != Shore1.end()) degree--;
        assert(degree > 0);
        IloExpr expr(_LMILP->_env);
        for (int k : K) {
            expr -= degree*_LMILP->_y[s][k];
            for (auto i : Shore1) for (auto j : Shore2)
                expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];
        }

        IloRange* R = new IloRange(_LMILP->_env, 0, expr, IloInfinity, LazyMILP::FlowCut.c_str());
        //cout << *R << endl;
        expr.end();
        return R;

    }

    IloRange* genSECCut(const vector<int>& Shore1, const vector<int>& Shore2, int s, int ks = -1) {
        assert(!Shore1.empty() && !Shore2.empty());

        vector<int> K;
        if (ks < 0)  for (int k = 0; k < _I->nvehicles(); ++k) K.push_back(k);
        else K.push_back(ks);
        int degree = 2;
        if (find(Shore2.begin(), Shore2.end(), initialdepotid()) != Shore2.end()) degree--;
        if (find(Shore2.begin(), Shore2.end(), finaldepotid()) != Shore2.end()) degree--;
        assert(degree > 0);
        assert(!isdepot(s));
        IloExpr expr(_LMILP->_env);
        for (int k : K) {
            expr -= degree * _LMILP->_y[s][k];
            for (auto i : Shore1) for (auto j : Shore2)
                expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];
        }

        IloRange* R = new IloRange(_LMILP->_env, 0, expr, IloInfinity, LazyMILP::SECCut.c_str());
        //cout << *R << endl;
        expr.end();
        return R;

    }

    IloRange* genMultiFlowCut(const vector<int>& Shore1, const vector<int>& Shore2, vector<int> Sources, vector< vector<int> >& K) {
        assert(!K.empty() && !Sources.empty());
        assert(K.size() == Sources.size());
        int degree = 2;
        if (find(Shore1.begin(), Shore1.end(), initialdepotid()) != Shore1.end()) degree--;
        if (find(Shore1.begin(), Shore1.end(), finaldepotid()) != Shore1.end()) degree--;
        assert(degree > 0);
        IloExpr expr(_LMILP->_env);

        auto n = Sources.size();
        for(int p=0;p<n;++p)
            for (auto k : K[p]) expr -= degree * _LMILP->_y[Sources[p]][k];

        for (int k = 0; k < _I->nvehicles(); ++k) {
            for (auto i : Shore1) for (auto j : Shore2)
                expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];
        }

        IloRange* R = new IloRange(_LMILP->_env, 0, expr, IloInfinity, "MultiFlow");

        expr.end();
        return R;

    }

    IloRange* genConnectFlowCut(const vector<int>& Shore1, const vector<int>& Shore2, int s, int t, int ks=-1) {
        assert(!Shore1.empty() && !Shore2.empty());

        vector<int> K;
        if (ks < 0)  for (int k = 0; k < _I->nvehicles(); ++k) K.push_back(k);
        else K.push_back(ks);


        int degree = 2;
        assert(find(Shore1.begin(), Shore1.end(), s) != Shore1.end());
        assert(find(Shore2.begin(), Shore2.end(), t) != Shore2.end());
        size_t n1 = count_if(Shore1.begin(), Shore1.end(), [&](int i) {return isdepot(i); });
        size_t n2 = count_if(Shore2.begin(), Shore2.end(), [&](int i) {return isdepot(i); });
        assert(n1+n2==2);
        if (n1 == n2) degree = 1;
        

        IloExpr expr(_LMILP->_env);

        for (int k : K) {
            expr -= degree * (_LMILP->_y[s][k] + _LMILP->_y[t][k]);
            for (auto i : Shore1) for (auto j : Shore2)
                expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];
        }

        IloRange* R = new IloRange(_LMILP->_env, -degree, expr, IloInfinity, LazyMILP::ConnectCut.c_str());

        expr.end();
        return R;

    }

    //Note. also refered as subcycle elimination constraints (SECs)
    IloRange* genSubCycleCut(const vector<int>& Shore1, const vector<int>& Shore2) {
        assert(!Shore1.empty() && !Shore2.empty());
        double maxSum = -DBL_MAX;
        int uMax = -1, vMax = -1, kMax=-1;
        for (int k = 0; k < _I->nvehicles(); ++k) {
            IloNumArray Y = getYVal(k);
            for (auto u : Shore1) if(_I->comp(k,u)){
                for (auto v : Shore2) if (_I->comp(k, v)) {
                    double currentSum = (double)(Y[u] + Y[v]);
                    if (currentSum > maxSum) {
                        maxSum = currentSum;
                        uMax = u;
                        vMax = v;
                        kMax = k;
                    }
                }
            }
            Y.end();
        }
        if (kMax < 0) return NULL;
        
        IloExpr expr(_LMILP->_env);
        expr -=  _LMILP->_y[uMax][kMax] + _LMILP->_y[vMax][kMax];
        for (auto i : Shore1) for (auto j : Shore2)
            expr += _LMILP->_x[_LMILP->getarcid(i, j, kMax)];
        

        IloRange* R = new IloRange(_LMILP->_env, -2, expr, IloInfinity, LazyMILP::SubCycleCut.c_str());

        expr.end();
        return R;

    }

    IloRange* genIncompatibilityCut(const vector<int>& Shore1, const vector<int>& Shore2, int s1, int s2) {
        assert(!Shore1.empty() && !Shore2.empty());
        int degree = 2;
        //if (find(Shore1.begin(), Shore1.end(), initialdepotid()) != Shore1.end()) degree--;
        //if (find(Shore1.begin(), Shore1.end(), finaldepotid()) != Shore1.end()) degree--;
        //assert(degree > 0);
        size_t n1 = count_if(Shore1.begin(), Shore1.end(), [&](int i) {return isdepot(i); });
        size_t n2 = count_if(Shore2.begin(), Shore2.end(), [&](int i) {return isdepot(i); });
        assert(n1 + n2 == 2);
        if (n1 == n2) degree = 1;
        IloExpr expr(_LMILP->_env);
        for (int k = 0; k < _I->nvehicles(); ++k) {
            expr -= degree * (_LMILP->_y[s1][k] + _LMILP->_y[s2][k]);
            for (auto i : Shore1) for (auto j : Shore2)
                expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];
        }

        IloRange* R = new IloRange(_LMILP->_env, 0, expr, IloInfinity, LazyMILP::InCompCut.c_str());

        expr.end();
        return R;

    }

    IloRange* genInfPathCut(const vector<int>& NodeSet) {
        assert(!NodeSet.empty());

        vector<int> Path = NodeSet;
        if (isdepot(Path.front())) Path.erase(Path.begin());
        if (isdepot(Path.back())) Path.pop_back();

        IloExpr expr(_LMILP->_env);
        for (int k = 0; k < _I->nvehicles();++k) for (int l = 0; l < Path.size() - 1; ++l) {
            int i = Path[l];
            int j = Path[l + 1];
            expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];
        }

        
        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, (int)Path.size() - 2, LazyMILP::InfPathCut.c_str());


        expr.end();
        return R;

    }

    IloRange* genMatchingCut(const vector<int>& H, const vector< pair<int,int> >& T) {
        assert(!H.empty() && !T.empty());
        assert(T.size() % 2 == 1);

        int n = 0;// count_if(H.begin(), H.end(), [&](int i) {return isdepot(i); });

        IloExpr expr(_LMILP->_env);
        for (int k = 0; k < _I->nvehicles(); ++k) {
            for (auto i : H) /*if(!isdepot(i))*/ expr -=  _LMILP->_y[i][k];
            for (auto& a : T)
                expr += _LMILP->_x[_LMILP->getarcid(a.first, a.second, k)];
            for (auto i : H) for (auto j : H) if (j < i)
                expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];
        }

        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, (T.size()-1)/2.0+n, LazyMILP::MatchCut.c_str());
        //Node: mooded by adding n related to the presence of any depot: It cutted some optimal solution

        expr.end();
        return R;

    }

    IloRange* genMatchingCut(const vector<int>& H, const vector< pair<int, int> >& T, const vector<int> &K) {
        assert(!H.empty() && !T.empty());
        assert(T.size() % 2 == 1);

        int n = 0;// count_if(H.begin(), H.end(), [&](int i) {return isdepot(i); });

        IloExpr expr(_LMILP->_env);
        for (int k : K) {
            for (auto i : H) /*if(!isdepot(i))*/ expr -= _LMILP->_y[i][k];
            for (auto& a : T)
                expr += _LMILP->_x[_LMILP->getarcid(a.first, a.second, k)];
            for (auto i : H) for (auto j : H) if (j < i)
                expr += _LMILP->_x[_LMILP->getarcid(i, j, k)];
        }
        string name = LazyMILP::MatchCut;// +"-" + to_string(K.size());
        IloRange* R = new IloRange(_LMILP->_env, -IloInfinity, expr, (T.size() - 1) / 2.0 + n, name.c_str());
        //Node: mooded by adding n related to the presence of any depot: It cutted some optimal solution

        expr.end();
        return R;

    }

    vector<int> SetVehicleCompatible(const vector<int>& R) {
        vector<int> V;
        for (int k = 0; k < _I->nvehicles(); ++k) V.push_back(k);
        for (auto i : R) {
            auto last = remove_if(V.begin(), V.end(), [&](int k) {return !_I->comp(k, i); });
            //if (last != V.end()) cout << *last << " - " << i << endl;
            V.erase(last, V.end());
            if (V.empty()) 
                break;
        }
        return V;
    }

    void setXVal(int i, int j, int k, double val) { _XVal[_LMILP->getarcid(i,j,k)] = val;}
    double getXVal(int i, int j, int k) { 
        if (i==j) return 0.0;
        try {
            return _XVal[_LMILP->getarcid(i, j, k)];
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
            for (int k = 0; k < _I->nvehicles(); ++k) val += getXVal(u, v, k);
        return val;
    }


    double getXVal(const vector<int> H, const vector<int> T) {
        double val = 0;
        for (auto u : H) for (auto v : T) if (u != v)
            for (int k = 0; k < _I->nvehicles(); ++k) val += getXVal(u, v, k);
        return val;
    }


    double score(int i) {
        if (i >= 1 && i < _I->nnodes() - 1) return _I->node(i)->score();
        else return 0;
    }


    bool isInteger(int i, int j, int k) {
        double val = getXVal(i,j,k);
        return val <= _eps || val>=1.0 - _eps;
    }

    bool isInteger(double threshold=EPS) {
        for (int l = 0; l < _XVal.getSize(); ++l)
            if (_XVal[l] > threshold || _XVal[l] < 1.0 - threshold) return false;
        return true;
    }

    double percinteger() {
        int N = 0;
        for (int l = 0; l < _XVal.getSize(); ++l)
            if (_XVal[l] <= EPS || _XVal[l] >= 1.0 - EPS) N++;
        return (double)N / (double)_XVal.getSize();
    }

    double integralitymeasure(double threshold = EPS) {
        int NInt = 0;
        int N = 0;
        for (int l = 0; l < _XVal.getSize(); ++l) {
            double val = _XVal[l];
            if (val > EPS && val < 1.0 - EPS) {
                N++;
                if (val <= threshold || val >= 1.0 - threshold) NInt++;
            }
        }

        return (double)NInt/(double)N;
    }

    double score(const vector<int>& R) {
        return accumulate(R.begin(), R.end(), 0.0, [&](double a, int i) {
            return a + _I->node(i)->score();
            });
    }

    double sumArcFlow(vector<int>& V) {
        double sum = 0.0;
        for(int k=0;k<_I->nvehicles();k++)
            for (auto i : V) for (auto j : V) if (i < j) sum += getXVal(i, j, k);
        return sum;
    }

    double length(const vector<int>& R) {
        double L = 0.0;
        bool isreversed = false;
        if (R.front() == _I->nnodes() - 1) isreversed = true;
        int i = R.front();
        //assert(i == 0 || i >= _I->nnodes() - 1);
        for (auto j : R) if (i != j) {
 
            double dij = 0;
            if (isreversed) dij = _I->distance(_I->node(j), _I->node(i)) + _I->node(i)->serviceTime();
            else dij = _I->distance(_I->node(i), _I->node(j)) + _I->node(j)->serviceTime();

            L += dij;
            
            i = j;
        }
        return L;
    }


    double dev(const vector<int>& R) {
        return fabs(score(R) - getmu());
    }

    vector<int> getRandomPath(int k, int s = -1, double threshold = EPS);

    vector<int> getRandomInfPath( int s , int ks=-1);

    vector< vector<int> > getHeuristicRoutes();

    void extendRandomPath(vector<int>& Path, int k, double threshold = EPS) {
        //vector<bool> _RStatus(_nnodes, false);
        //for (auto i : Path) _RStatus[i] = true;
        if (Path.empty()) return;
        if (Path.size()>1 && isdepot(Path.back())) return;

        IloNumArray Y = getYVal(k);
        vector<bool> _RStatus(_nnodes, true);
        for (int i = 0; i < _nnodes; ++i) if (/*!_I->comp(k, i)*/Y[i]>EPS) _RStatus[i] = false;
        for (auto i : Path) _RStatus[i] = true;

        do {
            int j = Path.back();
            _RStatus[j] = true;

            vector<int> Candidates;
            for (int i = 0; i < _nnodes; ++i) if (!_RStatus[i] && getXVal(i, j, k) > threshold) 
                Candidates.push_back(i);
            

            int i = finaldepotid();
            if (!Candidates.empty()) {
                i=*RouletteWheelSelection(Candidates.begin(), Candidates.end(), [&](int i) {return  Y[i]/*getXVal(i, j, k)*/;  });
                Path.push_back(i);
            }
            else {
                break;
            }



        } while (!isdepot(Path.back()));

        //auto itd = find(Path.begin(), Path.end(), finaldepotid());
        //if (itd != Path.end() && itd != Path.begin() && *itd != Path.back()) Path.erase(itd);

        //auto ito = find(Path.begin(), Path.end(), initialdepotid());
        //if (ito != Path.end() && ito != Path.begin() && *ito != Path.back()) Path.erase(ito);

        Y.end();
    }

    vector<int> getMegaCycle(int k, double threshold=EPS);

    bool hasArc(int i, int j) {
        bool isconnected = false;
        for (int k = 0; k < _I->nvehicles() && !isconnected; ++k)
            isconnected = (getXVal(i, j, k) > EPS);

        return isconnected;
    }

    bool hasConflict(vector<int>& V) {
        for (auto u : V) for (auto v : V) if (u < v) {
            bool arecomp = false;
            for (int k = 0; k < _I->nvehicles() && !arecomp; ++k)
                arecomp = (_I->comp(k, u) && _I->comp(k, v));
            if (!arecomp) return true;
        }
        return false;
    }

    vector< pair<int, int> > getConflictNodes(vector<int>& V) {
        vector< pair<int, int> > C;
        for (auto u : V) for (auto v : V) if (u < v) {
            bool arecomp = false;
            for (int k = 0; k < _I->nvehicles() && !arecomp; ++k)
                arecomp = (_I->comp(k, u) && _I->comp(k, v));
            if (!arecomp) C.push_back(pair<int,int>(u,v));
        }

        return C;
    }

    vector<int> getCycle(int s);

    vector<int> bfs(double** RNetwork, int n, int s, int t, double threshold=EPS);

    vector<int> bfs(double** RNetwork, int n, vector<int>& S, vector<int>& T, double threshold = EPS);

    pair< vector<int>, vector<int> > getShoresMinCut(int s, int t=-1, int k=-1);

    pair< vector<int>, vector<int> > getShoresMinCut(vector<int>& S, vector<int>& T, int k = -1);

    double getFlowValue(const vector<int>& Shore1, const vector<int>& Shore2) {
        double flow = 0.0;
        for (auto u : Shore1) for (auto v : Shore2) for (int k = 0; k < _I->nvehicles(); ++k) flow += fmin(1.0, getXVal(u, v, k));
        return flow;
    }

    double getFlowValue(const vector<int>& Shore1, const vector<int>& Shore2, int k) {
        double flow = 0.0;
        for (auto u : Shore1) for (auto v : Shore2) flow += fmin(1.0, getXVal(u, v, k));
        return flow;
    }

    vector<int> getRoute(int k);

    vector< vector<int> > getRoutes(int k);

    pair<double, double> getRouteData(const vector<int>& R)
    {
        //Warning: Route may be in reverse order
        double T = 0.0;
        double S = 0.00;
        bool isreversed=false;
        if(R.front()==_I->nnodes()-1) isreversed=true;
        int i = R.front();
        assert(i == 0 || i >= _I->nnodes() - 1);
        for (auto j : R) if (i != j) {
            i = min(i, _I->nnodes() - 1);
            j = min(j, _I->nnodes() - 1);

            double dij=0;
            if(isreversed) dij=_I->distance(_I->node(j), _I->node(i))+ _I->node(i)->serviceTime();
            else dij=_I->distance(_I->node(i), _I->node(j))+ _I->node(j)->serviceTime();

            T += dij;
            S += _I->node(j)->score();

            i = j;
        }
        assert(i == 0 || i == _I->nnodes() - 1);
        return pair<double, double>(T, S);
    }

    vector<int> repairroute(const vector<int>& R, int k, double MinScore=0.0);

    void removeDuplicateNodes(vector< vector<int> >& Routes);

    void adjustRoutesLength(vector< vector<int> >& Routes);

    void optimizeRoutesLength(vector< vector<int> >& Routes);

    void improveRoutes(vector< vector<int> >& Routes, double target);

    void reorderRoutes(vector< vector<int> >& Routes);

    bool bestinsert(vector<int>& R, int s) {
        double l = length(R);
        auto bestit = R.end();
        double bestdelta = DBL_MAX;
        int i = R.front();
        for (auto it = R.begin(); it != R.end(); ++it) if(i!=*it){
            assert(!isfinaldepot(i));
            int j = *it;
            double dij = _I->distance(_I->node(i), _I->node(j));
            double dis = _I->distance(_I->node(i), _I->node(s));
            double dsj = _I->distance(_I->node(s), _I->node(j));
            double delta = dis + dsj - dij+_I->node(s)->serviceTime();
            //assert(delta >= -EPS);

            if (delta < bestdelta && l+delta<=_I->tmax()/*-EPS*/) {
                bestdelta = delta;
                bestit = it;
            }

            i = j;
        }
        
        if (bestit != R.end()) {
            bestit = R.insert(bestit, s);
            //assert(bestdelta >= -EPS);

        }

        return bestit != R.end();
    }

    int swapinsert(vector<int>& R, int s) {
        double F = fitness(R);
        vector<int> Candidate;
        copy_if(R.begin(), R.end(), back_inserter(Candidate), [&](int i) { return !isdepot(i) && _I->node(i)->score() < _I->node(s)->score(); });
     
        if (Candidate.empty()) return s;

        sort(Candidate.begin(), Candidate.end(), [&](int i, int j) {return _I->node(i)->score() < _I->node(j)->score(); });

        for (auto i : Candidate) {
            auto it = find(R.begin(), R.end(), i);
            assert(it != R.end());
            *it = s;
            double l = length(R);
            double newF = fitness(R);
            if (l <= _I->tmax() &&
                (getOF() == 1 || newF < F))
                return i;
            else *it = i;
        }

        return s;
    }

    vector<int> longestPath(int source, int k);

    pair< IloNumVarArray*, IloNumArray* > extractSolution();
};

class CutPool : public vector< IloRange* > {

    std::function<IloNum(IloExpr&)> _evalExpr;
    double _mindisttocut;

public:
    CutPool(const std::function<IloNum(IloExpr&)>& eval, double mindist=0.01) : vector< IloRange* >(), _evalExpr(eval), _mindisttocut(mindist){}
    ~CutPool() {}

    bool isvalidcut(IloRange* R) {
		string name = R->getName();
        // We check if the cut is enabled from the defined MACROS
        if (!USE_GSEC && name == LazyMILP::CycleCut) return false;
		if (!USE_CC && (name == LazyMILP::FlowCut || name == LazyMILP::SECCut)) return false;
		if (!USE_MC && name == LazyMILP::MatchCut) return false;
		if (!USE_CCC && name == LazyMILP::CycleCoverCut) return false;
		if (!USE_ICC && name == LazyMILP::TmaxCut) return false;
		if (!USE_IPC && name == LazyMILP::PathCut) return false;
		if (!USE_IPCC && name == LazyMILP::PathCoverCut) return false;
		if (!USE_LC && name == LazyMILP::LogCut) return false;
		return true;

    }

    double DistanceToCut(IloRange* R) {
        if (R == NULL) return 0.0;
        IloExpr expr = R->getExpr();
        IloNum val = _evalExpr(expr);
        IloNum lhs = R->getLB();
        IloNum rhs = R->getUB();

        double diff = fmax((lhs - val), (val - rhs));
		//expr.end();
  //      return diff;

        IloExpr::LinearIterator it = expr.getLinearIterator();
        double qsum = 0.0;
        while (it.ok()) {
            if (it.getVar().getUB()>EPS)
                qsum += pow(it.getCoef(), 2);
            ++it;
        }
        //if (qsum < 1.0 - EPS) {
        //    cout << R->getName() << endl;
        //    cout << *R << endl;
        //}
        expr.end();
        if (qsum </* 1.0 -*/ EPS) return 0;
        assert(qsum >= 1.0);
        qsum = sqrt(qsum);
        

        double dist = diff / qsum;
        //if(dist<0) cout << R->getName() <<  ": " << diff << " - " << qsum << " - " << dist << endl;
        assert(fabs(diff) >= fabs(dist));

        
        return dist;
    }

    bool push(IloRange* R) {
        if (R == NULL) return false;

        double dist = DistanceToCut(R);
        bool add = isvalidcut(R) && (dist > _mindisttocut);
        
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


class MyCallback : public IloCplex::Callback::Function {
private:

    struct edge {
        int i;
        int j;
        double w;

        edge(int s, int t, double x) : i(s), j(t), w(x) {};
    };

    Instance* _I;

    bool _enablelog;
    vector < pair<string, int> >* _Log; //One per threads

    LazyMILP* _LMILP;

    vector< pair<int, int> > _ConflictPairs;

    bool containsConflicts(const vector<int>& S) {
        vector<bool> status(_I->nnodes(), false);
        for (auto i : S) status[i] = true;
        auto it = find_if(_ConflictPairs.begin(), _ConflictPairs.end(),
            [status](pair<int, int> a) {return status[a.first] && status[a.second]; });
       // assert(it == _ConflictPairs.end() || !getConflictNodes(S).empty());
        return it != _ConflictPairs.end();
    }

    vector< pair<int, int> > getConflictNodes(const vector<int> S) {
        vector< pair<int, int> > C;
        vector<bool> status(_I->nnodes(), false);
        assert(status.size() == _I->nnodes());
        for (auto i : S) status[i] = true;
        copy_if(_ConflictPairs.begin(), _ConflictPairs.end(), back_inserter(C), 
            [status](pair<int, int> a) {return status[a.first] && status[a.second]; });
       // assert(!C.empty());
        return C;
    }

	void getCycleCuts(CutPool& Cuts, CplexSolution* S, double target);

    void addBranches(const IloCplex::Callback::Context& context);

    void addLazyCuts(const IloCplex::Callback::Context& context);

    void addUserCuts(const IloCplex::Callback::Context& context);

    void searchSolutionFromLP(const IloCplex::Callback::Context& context);

    bool addSolution(const IloCplex::Callback::Context& context, CplexSolution* S);

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

    void collectLogicalConstraints(CutPool& Cuts, CplexSolution* S, double bestbound);

    void collectCutsOnRoutes(CutPool& Cuts, const vector< vector<int> >& Routes, CplexSolution* S, double target);

    void collectCutsOnFlow(CutPool& Cuts, CplexSolution* S, double target, double threshold = EPS);

    void collectCutsOnFlowPerVehicles(CutPool& Cuts, CplexSolution* S, double target);

    void collectExtraCutsOnFlowPerVehicles(CutPool& Cuts, CplexSolution* S, double target);

    void collectCutsOnMultiFlow(CutPool& Cuts, CplexSolution* S, double threshold = EPS);

    void collectCutsOnConflicts(CutPool& Cuts, CplexSolution* S, double target, double threshold = EPS);

    void collectCutsOnConflictsPath(CutPool& Cuts, CplexSolution* S, double target, double threshold = EPS);


    void PrintCut(IloRange* Cut, const IloCplex::Callback::Context& context) {
            double val = context.getCandidateValue(Cut->getExpr());
            double lhs = Cut->getLB();
            double rhs = Cut->getUB();
            cout << ": " << lhs << "<=" << val << "<=" << rhs << endl;
            cout << *Cut << endl;
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


    //The Matching Constraints (Simple Comb?)
    int findparent(int i, vector<int>& parent) {
        if (parent[i] == -1) return i;
        else return parent[i] = findparent(parent[i], parent);
    }
    void unite(int i, int j, vector<int>& parent, vector<int>& rank) {
        int s1 = findparent(i,parent);
        int s2 = findparent(j,parent);
        if (s1 != s2) {
            if (rank[s1] < rank[s2])  parent[s1] = s2;
            else if (rank[s1] > rank[s2]) parent[s2] = s1;
            else {
                parent[s2] = s1;
                rank[s1] += 1;
            }
        }
    }
    void collectMatchingCuts(CutPool& Cuts, CplexSolution* S, const double threshold=1.0-EPS);
    
    void collectMatchingCutsPerVehicles(CutPool& Cuts, CplexSolution* S);

    void collectMatchingCutsV2(CutPool& Cuts, CplexSolution* S, double target, const double threshold = EPS);

    void collectMatchingCuts(CutPool& Cuts, CplexSolution* S, vector<int>& K, const double threshold);

    void collectTreeCuts(CutPool& Cuts, CplexSolution* S, vector<int>& K, double target);

    void collectTreeCuts(CutPool& Cuts, CplexSolution* S, double target);

    double minSpanningTreeLength(const vector<int>& Nodes, const double threshold = EPS);

    double minSpanningTreeLength(vector<edge>& E);

    void updatelog(const IloCplex::Callback::Context& context, CutPool& Cuts);

public:
    // Constructor
    MyCallback(LazyMILP *LMILP);

    ~MyCallback() { delete[] _Log; }

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

    void saveLog();

    void saveGraph(const IloCplex::Callback::Context& context) {
        string filename = "Graph.txt";
        ofstream output(filename.c_str());
        assert(output.is_open());
        output.setf(ios::left);
        output.precision(2);
        output.setf(ios::fixed, ios::floatfield);
        vector<string> Color = { "red", "blue", "green", "brown", "black"};
        assert(_I->nvehicles() <= Color.size()-1);

        CplexSolution* S = NULL;
        if (context.inRelaxation() || context.inBranching()) S = new CplexSolution(_LMILP, [&context](IloNumVarArray& X, IloNumArray& V) { context.getRelaxationPoint(X, V); });
        if (context.inCandidate()) S = new CplexSolution(_LMILP, [&context](IloNumVarArray& X, IloNumArray& V) { context.getCandidatePoint(X, V); });
        output << "Graph {" << endl;

        output << S->initialdepotid() << "[shape=box];" << endl;
        output << S->finaldepotid() << "[shape=box];" << endl;

        //Individual graph
        for (int k = 0; k < _I->nvehicles(); ++k) {
            for (int i = 0; i < _I->nnodes(); ++i) for (int j = 0; j < i; ++j) if(S->getXVal(i,j,k)>EPS){
                output << i << " -- " << j << "[ label=" << "\"" << S->getXVal(i, j, k) << "\", color=" << Color[k] << "];" << endl;
            }
        }

        //General graph
        for (int i = 0; i < _I->nnodes(); ++i) for (int j = 0; j < i; ++j) {
            double x = 0.0;
            for (int k = 0; k < _I->nvehicles(); ++k) x += S->getXVal(i, j, k);
            if(x>EPS)
                output << i << " -- " << j << "[ label=" << "\"" << x << "\", color=" << Color.back() << "];" << endl;
        }

        output << "}" << endl;


        output.close();
        delete S;
        cout << "Graph saved to " << filename << endl;
        //exit(1000);
    }

    void saveSolGraph(const IloCplex::Callback::Context& context) {
        string filename = "SolGraph.txt";
        ofstream output(filename.c_str());
        assert(output.is_open());
        output.setf(ios::left);
        output.precision(2);
        output.setf(ios::fixed, ios::floatfield);
        vector<string> Color = { "red", "blue", "green", "brown", "black" };
        assert(_I->nvehicles() <= Color.size() - 1);

        CplexSolution* S = new CplexSolution(_LMILP, [&context](IloNumVarArray& X, IloNumArray& V) { context.getCandidatePoint(X, V); });
        output << "Graph {" << endl;

        output << S->initialdepotid() << "[shape=box];" << endl;
        output << S->finaldepotid() << "[shape=box];" << endl;

        //Individual graph
        for (int k = 0; k < _I->nvehicles(); ++k) {
            for (int i = 0; i < _I->nnodes(); ++i) for (int j = 0; j < i; ++j) if (S->getXVal(i, j, k) > EPS) {
                output << i << " -- " << j << "[ label=" << "\"" << S->getXVal(i, j, k) << "\", color=" << Color[k] << "];" << endl;
            }
        }

        //General graph
        for (int i = 0; i < _I->nnodes(); ++i) for (int j = 0; j < i; ++j) {
            double x = 0.0;
            for (int k = 0; k < _I->nvehicles(); ++k) x += S->getXVal(i, j, k);
            if (x > EPS)
                output << i << " -- " << j << "[ label=" << "\"" << x << "\", color=" << Color.back() << "];" << endl;
        }

        output << "}" << endl;


        output.close();
        delete S;
        cout << "Graph saved to " << filename << endl;
        //exit(1000);
    }

    // The main callback function
    void invoke(const IloCplex::Callback::Context& context);


};



