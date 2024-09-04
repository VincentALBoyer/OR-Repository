#include "Solver.h"

inline void Solver::startTimer() { _start = chrono::system_clock::now(); }

double Solver::timeLeft() { return fmax(0, _TimLim - chrono::duration_cast<std::chrono::seconds>(chrono::system_clock::now() - _start).count()); }

inline void Solver::stopTimer() { _end = chrono::system_clock::now(); }

inline string Solver::PadNumber(int num, int w, char c)
{
	std::ostringstream ss;
	ss << std::setw(w) << std::setfill(c) << num;
	return ss.str();
}

inline string Solver::RedString(string s, int w) {
	if (s.size() > w) {
		string f = "...";
		int ww = (int)floor((w - f.size()) / 2.0);
		return s.substr(0, ww) + f + s.substr(s.size() - ww, s.size());
	}
	else return s;
}

bool Solver::fwdImprovementPositioning(Solution* S, int opid)
{
	operation* o = S->getop(opid);
	assert(S->isopinroute(o));
	bool moved = false;
	
	S->commit();
	operation* nexto = S->nextvehicleop(o);
	while (S->nextvehicleop(o) != NULL && _I->nodetype(S->getid(o)) == _I->nodetype(S->getid(S->nextvehicleop(o)))) {
		S->fwdmove(o);
		if (S->update(false)) {
			int d = 0;
			for (int i = 0; i < S->nop(); ++i) d += S->deltast(S->getop(i));
			if (d < 0) {
				S->commit();
				moved = true;
			}
		}
	}
	S->revert();
	
	return moved;
}

bool Solver::bwdImprovementPositioning(Solution* S, int opid)
{
	operation* o = S->getop(opid);
	assert(S->isopinroute(o));
	bool moved = false;

	S->commit();
	operation* prevo = S->prevvehicleop(o);
	while (S->prevvehicleop(o) != NULL && _I->nodetype(S->getid(o)) == _I->nodetype(S->getid(S->prevvehicleop(o)))) {
		S->bwdmove(o);
		if (S->update(false)) {
			int d = 0;
			for (int i = 0; i < S->nop(); ++i) d += S->deltast(S->getop(i));
			if (d < 0) {
				S->commit();
				moved = true;
			}
		}
	}
	S->revert();

	return moved;
}

void Solver::setLB(Instance* I, LBAlgo A)
{
	double LB = 1;
	if (A == LBAlgo::CPLoad) {
		try {
			int L = 0;
			for (int g = 0; g < I->ngroups(); ++g) L += I->maxtourlength(g);

			IloEnv env;
			IloModel model(env);
			IloCP cp(model);

			IloIntervalVarArray O(env);
			IloCumulFunctionExpr C(env);
			for (int i = 0; i < I->nnodes(); ++i) {
				string label = "op_" + to_string(i);
				int length = 0;
				if (I->ispickupnode(i)) {
					length = I->travellingtime(i, I->nextnodeid(i));
				}
				IloIntervalVar op(env, label.c_str());
				op.setLengthMin(length);
				op.setEndMax(I->planhor());
				op.setPresent();
				O.add(op);

				C += IloPulse(op, I->groupsize(I->groupid(i)));
			}

			IloIntervalVar S(env);
			S.setStartMax(0);
			S.setLengthMax(I->planhor());
			model.add(IloSpan(env, S, O));

			IloIntVar N(env, 1, I->ngroups());
			

			//Objective
			model.add(IloMinimize(env, N));

			//Vehicle capacity
			model.add(C <= I->vcapacity()*N);

			//Group precedence constraints
			for (int g = 0; g < I->ngroups(); ++g) {
				int o = I->firstgroupnode(g);
				while (!I->islastnode(o)) {
					int nexto = I->nextnodeid(o);
					if (I->ispickupnode(o)) {
						assert(I->isdeliverynode(nexto));
						if (!I->islastnode(nexto)) {
							int nextnexto = I->nextnodeid(nexto);
							int length = I->travellingtime(nexto, nextnexto);
							model.add(IloEndBeforeStart(env, O[o], O[nextnexto], length));
						}
					}
					o = nexto;
				}

				assert(I->islastnode(o));
				assert(I->ispickupnode(I->prevnodeid(o)));
				model.add(IloEndBeforeStart(env, O[I->prevnodeid(o)], O[I->firstgroupnode(g)], -I->maxtourlength(g)));
			}

			cp.setParameter(IloCP::TimeLimit, 60);
			cp.setParameter(IloCP::FailLimit, IloIntMax);
			cp.setParameter(IloCP::RelativeOptimalityTolerance, 0.01);
			cp.setParameter(IloCP::Workers, 1);
			cp.setParameter(IloCP::SearchType, IloCP::Restart);

			cp.solve();

			LB = ceil(cp.getObjBound());

			cout << "LB= " << LB << endl;

			env.end();

		}
		catch (IloException& e) {
			cout << e << endl;
			exit(1001);
		}
	}
	else if (A == LBAlgo::CP) {
		try {
			int L = 0;
			for (int g = 0; g < I->ngroups(); ++g) L += I->maxtourlength(g);

			IloEnv env;
			IloModel model(env);
			IloCP cp(model);

			IloIntervalVarArray O(env);
			IloCumulFunctionExpr C(env);
			for (int i = 0; i < I->nnodes(); ++i) {
				string label = "op_" + to_string(i);
				int length = 0;
				if (I->ispickupnode(i)) {
					length = I->travellingtime(i, I->nextnodeid(i));
				}
				IloIntervalVar op(env, label.c_str());
				op.setLengthMin(length);
				op.setPresent();
				O.add(op);

				C += IloPulse(op, I->groupsize(I->groupid(i)));
			}

			IloIntervalVar S(env);
			S.setStartMax(0);
			S.setLengthMax(L);
			model.add(IloSpan(env, S, O));

			IloIntVar N(env, 1, (int)ceil((double)L/(double)I->planhor()));
			model.add(I->planhor()* N >= IloLengthOf(S));
			model.add( IloLengthOf(S) >= I->planhor() * (N-1));

			//Objective
			model.add(IloMinimize(env, N));

			//Vehicle capacity
			model.add(C <= I->vcapacity());

			//Group precedence constraints
			for (int g = 0; g < I->ngroups(); ++g) {
				int o = I->firstgroupnode(g);
				while (!I->islastnode(o)) {
					int nexto = I->nextnodeid(o);
					if (I->ispickupnode(o)) {
						assert(I->isdeliverynode(nexto));
						if (!I->islastnode(nexto)) {
							int nextnexto = I->nextnodeid(nexto);
							int length = I->travellingtime(nexto, nextnexto);
							model.add(IloEndBeforeStart(env, O[o], O[nextnexto], length));
						}
					}
					o = nexto;
				}

				assert(I->islastnode(o));
				assert(I->ispickupnode(I->prevnodeid(o)));
				model.add(IloEndBeforeStart(env, O[I->prevnodeid(o)], O[I->firstgroupnode(g)], -I->maxtourlength(g)));
			}

			cp.setParameter(IloCP::TimeLimit, 60);
			cp.setParameter(IloCP::FailLimit, IloIntMax);
			cp.setParameter(IloCP::RelativeOptimalityTolerance, 0.01);
			cp.setParameter(IloCP::OptimalityTolerance, 1);
			cp.setParameter(IloCP::Workers, 1);
			cp.setParameter(IloCP::SearchType, IloCP::Restart);

			cp.solve();

			LB = ceil(cp.getObjBound());

			cout << "LB= " << LB << endl;

			env.end();


		}
		catch (IloException& e) {
			cout << e << endl;
			exit(1001);
		}
	}
	else if (A == LBAlgo::Heuristic) {

		LB = 1;


	}
	else if (A == LBAlgo::LP) {
		try {

		}
		catch (IloException& e) {
			cout << e << endl;
			exit(1001);
		}
	}
	else {
		cout << "Error: Selected UB Algorithm Unknown" << endl;
	}

	I->setLB(LB);
}

Solver::Solver(Instance* I, string solvername)
{
	_I = I;
	_name = solvername;
	_gap = 1.0;
	reset();
}

Solver::~Solver()
{
}

void Solver::reset()
{
	_TimLim = _I->TimLim();
	_start = _end = chrono::system_clock::now();
	
}

void Solver::setTimLim(double val)
{
	_TimLim = (int)fmin(_I->TimLim(), val);
}

double Solver::cputime()
{
	return chrono::duration_cast<std::chrono::seconds>(_end - _start).count();
}

void Solver::solve(Solution* S)
{
	initsolver(S);

	startTimer();

	solvemethod();

	stopTimer();
}

void Solver::save(string outputfile)
{
	ofstream output;

	int width = 20;
	output.open(outputfile.c_str(), ios::app);

	output.setf(ios::left);
	output.precision(2);
	output.setf(ios::fixed, ios::floatfield);
	
	output << " ";
	if (output.tellp() == 1) {
		output << setw(width) << "Date" << setw(width) << "Time" << setw(40) << "Instance" << setw(width) << "Solver" << setw(width) << "Fitness" << setw(width) << "Gap" << setw(width) << "CPU_time" << setw(width) << "Feas";
		output << setw(width) << "#_spots" << setw(width) << "#_nodes" << setw(width) << "#_groups" << setw(width) << "#_tours" << setw(width) << "QoS" << setw(width) << "VCap" << setw(width) << "LB" << setw(width) << "Peek_Load" << setw(width) << "Av_Load" ;
		output << setw(width) << "SolQoS" << setw(width) << "AvQoS" << setw(width) << "MinOpV" << setw(width) << "MaxOpV" << endl;
		output << " ";
	}

	Solution* S = getSolution();
	string fitness = "n/a";
	string feas = "n/a";
	string peakLoad = "n/a";
	string AvLoad = "n/a";
	string SolQoS = "n/a";
	string AvQoS = "n/a";
	string MinOpV = "n/a";
	string MaxOpV = "n/a";
	if (S != NULL) {
		fitness = to_string(S->fitness());
		if (S->isFeasible()) feas = "yes";
		else feas = "no";
		peakLoad = to_string(S->peekLoad());
		AvLoad = to_string(S->averageLoad());
		SolQoS = to_string(S->QoS());
		AvQoS = to_string(S->AvQoS());
		MinOpV = to_string(S->MinNopVehicles());
		MaxOpV = to_string(S->MaxNopVehicles());
	}
	
	std::time_t end_time = std::chrono::system_clock::to_time_t(_end);
	std::tm* now = std::localtime(&end_time);

	string date = PadNumber(now->tm_mday) + "/" + PadNumber(now->tm_mon+1) + "/" + to_string(now->tm_year + 1900);
	string time = PadNumber(now->tm_hour) + ":" + PadNumber(now->tm_min);

	output << setw(20) << date << setw(20) << time << setw(40) << RedString(_I->instancename(),40) << setw(width) << RedString(_name,width) << setw(width) << fitness << setw(width) << gap() << setw(width) << cputime() << setw(width) << feas;
	output << setw(width) << _I->nspots() << setw(width) << _I->nnodes() << setw(width) << _I->ngroups() << setw(width) << _I->ntours() << setw(width) << _I->qos();
	output << setw(width) << _I->vcapacity() << setw(width) << _I->LB() << setw(width) << peakLoad << setw(width) << AvLoad;
	output << setw(width) << SolQoS << setw(width) << AvQoS << setw(width) << MinOpV << setw(width) << MaxOpV << endl;

	output.close();
	delete S;
}
