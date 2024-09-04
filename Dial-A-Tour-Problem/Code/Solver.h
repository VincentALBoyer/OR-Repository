#pragma once

#include "Solution.h"

enum class LBAlgo {CPLoad, CP, Heuristic, LP};

class Solver
{
protected:
	Instance* _I;
	string _name;

	int _TimLim;

	double _gap;

    chrono::time_point<chrono::system_clock> _start;
    chrono::time_point<chrono::system_clock> _end;

	void startTimer();

	void stopTimer();

	virtual void solvemethod() = 0;
	virtual void initsolver(Solution* S = NULL) = 0;

	string PadNumber(int num, int w = 2, char c = '0');
	string RedString(string s, int w = 20);

	bool fwdImprovementPositioning(Solution* S, int oid);

	bool bwdImprovementPositioning(Solution* S, int opid);

public:

	Solver(Instance *I, string solvername);

	~Solver();

	Instance* instance() { return _I; }

	void reset();

	void setTimLim(double val);

	double cputime();

	double timeLeft();

	double gap() { 
		_gap = 1.0;
		Solution* S = getSolution();
		if (S != NULL && S->fitness() >=1) {
			_gap = ((double)S->fitness() - _I->LB()) / (double)S->fitness();
		}
		delete S;
		return ceil(_gap*10000)/10000.0; 
	}

	void solve(Solution *S=NULL);

	string name() { return _name; }

	virtual Solution* getSolution()=0;

	void save(string outputfile = "Output/test.txt");

	static void setLB(Instance* I, LBAlgo A=LBAlgo::Heuristic);

};

