#pragma once
#include "Solution.h"

typedef int param_t;

class Solver{

protected:
	Instance* _I;
	string _name;
	double _timlim;
	double _mingap;
	int _threads;

	double _Bound;

	chrono::time_point<chrono::system_clock> _start;
	chrono::time_point<chrono::system_clock> _end;

	void startTimer() { _start = chrono::system_clock::now(); }
	void stopTimer() { _end = chrono::system_clock::now(); }

	virtual void solvemethod(Solution* S) = 0;

public:
	//parameters
	static const param_t TimLim = 0;
	static const param_t Gap = 1;
	static const param_t Threads = 2;


	Solver(Instance* I, string name);

	virtual ~Solver() {};

	void setparam(param_t PARAM, double value);

	string name() { return _name; }

	double cpuTime() const { return (double)chrono::duration_cast<std::chrono::seconds>(_end - _start).count(); }

	void solve(Solution* S);

	virtual double gap() = 0;
	virtual double value() = 0;

	virtual Solution* recoversolution()=0;
	void saveStat(Solution* S, int OF);

	double getBound() const { return _Bound; }

};

