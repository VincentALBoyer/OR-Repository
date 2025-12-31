#include "Solver.h"

Solver::Solver(Instance* I, string name)
{
	_I = I;
	_name = name;
	_start=_end= chrono::system_clock::now();
	_timlim = 60;
	_mingap = 0.0;
	_threads = 1;
	_Bound = 0;
}

void Solver::setparam(param_t PARAM, double value)
{
	if (PARAM == TimLim) _timlim = value;
	else if (PARAM == Gap) _mingap = value;
	else if (PARAM == Threads) _threads = (int)value;
}

void Solver::solve(Solution* S)
{
	startTimer();

	solvemethod(S);

	stopTimer();
}

void Solver::saveStat(Solution* S, int OF)
{
	// save statistics in a file
	string outputFile = "Output/NewStatistics.txt";
	string obj="n/a";
	double eval=0;
	if (OF == 1) {
		obj = "Max-Min";
		//value = S -> minScore();
		eval = S->maxDev();
	} else if (OF == 2) {
		obj = "Min-Max";
		eval = S -> minScore();
		//value = S->maxDev();
	}

	fstream output;
	output.open(outputFile.c_str(), ios::in);

	int w = 12;
	
	char SEP = '\\';
#ifdef __linux__
	SEP='/';
#endif
	string Name = _I->name();
	auto pos = Name.find_last_of(SEP);
	if(pos<Name.length()-1) Name = Name.substr(pos+1, Name.length());

	if (!output.is_open()) {
		ofstream output(outputFile.c_str());
		output.setf(ios::left);
		output.precision(2);
		output.setf(ios::fixed, ios::floatfield);
		output << setw(20) << "Instance" << setw(15) << "Solver" << setw(w) << "ObjF" << setw(w) << "OValue";
		output << setw(w) << "Eval" << setw(w) << "Gap" << setw(w) << "Bound" << setw(w) << "CPU_Time" << setw(w) << "Feas" << endl;
		output << setw(20) << Name << setw(15) << _name << setw(w) << obj << setw(w) << value() << setw(w);
		output << eval << setw(w) << gap() << setw(w) << getBound() << setw(w) << cpuTime() << setw(w)<< S->isFeasible()<< endl;
	} else {
		ofstream output(outputFile.c_str(), ios::app);
		output.setf(ios::left);
		output.precision(2);
		output.setf(ios::fixed, ios::floatfield);
		output << setw(20) << Name << setw(15) << _name << setw(w) << obj << setw(w) << value() << setw(w);
		output << eval << setw(w) << gap() << setw(w) << getBound() << setw(w) << cpuTime() << setw(w)<< S->isFeasible()<< endl;
	}
	output.close();
}

