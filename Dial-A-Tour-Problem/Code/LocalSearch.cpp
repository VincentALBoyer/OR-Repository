#include "LocalSearch.h"

void LocalSearch::solvemethod()
{
	if(_S==NULL) {
		_RepairHeur->setTimLim(timeLeft());
		_RepairHeur->solve();
		delete _S;
		_S = _RepairHeur->getSolution();
		assert(_S != NULL);
	}
	assert(_S->isFeasible());

	cout << setw(10) << "Imp" << setw(10) << "Curr" << setw(10) << "BestN" << setw(10) << "Best" << endl;

	Solution* CurrS = new Solution(_S);
	int k = 0;
	int timestep = 200;
	int timestepcount = 1;
	while (++k<=100 && timeLeft()>0.1) {
		
		Solution* best = getBestNeiborhood(CurrS);
		if (best->fitness() < _S->fitness()) {
			cout << setw(10) << "*";
			_S->copy(best);
			k = 0;
		}
		else {
			if (best->fitness() >= CurrS->fitness()) cout << setw(10) << "R";
			else cout << setw(10) << " ";
		}

		cout << setw(10) << CurrS->fitness() << setw(10) << best->fitness() << setw(10) << _S->fitness() << endl;

		
		if (best->fitness() >= CurrS->fitness()) {
			//We restart with new solution
			CurrS->copy(_S);
			perturbation(CurrS, 0.5);
			
		}
		else swap(CurrS, best);
		
		delete best;

		double CPUTime = _TimLim-timeLeft();
		if (CPUTime >= timestepcount * timestep) {
			cout << name() << ": " << ceil(100*CPUTime)/100.0 <<"s." << endl;
			timestepcount++;
		}

	}
	cout << name() << ": CPU=" << ceil(100 * (_TimLim - timeLeft())) / 100.0 << "s. - Gap=" << gap()*100 <<"%" << endl;
	delete CurrS;
}

void LocalSearch::initsolver(Solution* S)
{
	if (S != NULL) _S=new Solution(S);
	
}

Solution* LocalSearch::getBestNeiborhood(Solution* S)
{
	assert(S != NULL);
	//Solution* best = new Solution(S);
	Solution* best = new Solution(S);

	//Collecting the routes
	double* length = new double[S->nop()];
	vector<int> firstopid;
	for (int i = 0; i < best->nop(); ++i) {
		length[i] = 0.0;
		operation* o = S->getop(i);
		if (best->prevvehicleop(o) == NULL) {
			firstopid.push_back(i);
			double st = S->st(o);
			while (o != NULL) {
				length[i] = S->st(o)-st;
				o = S->nextvehicleop(o);
			}
			assert(length[i] > 0.0);
		}
	}

	random_shuffle(firstopid.begin(), firstopid.end());

	while (!firstopid.empty()) {
		vector<int>::iterator it = firstopid.begin();
		
		int i = *it;
		firstopid.erase(it);
		Solution* curr = new Solution(S);

		set<int> idtoremove;
		operation* o = curr->getop(i);
		assert(curr->prevvehicleop(o) == NULL);
		while(o != NULL) {
			idtoremove.insert(curr->getid(o));

			operation* ogr = curr->nextgroupop(o);
			while (ogr != NULL) {
				idtoremove.insert(curr->getid(ogr));
				ogr = curr->nextgroupop(ogr);
			}

			o = curr->nextvehicleop(o);
		}

		for (auto k : idtoremove) curr->removefromroute(curr->getop(k));
		assert(curr->update());
		
		_RepairHeur->solve(curr);
		delete curr;

		curr=_RepairHeur->getSolution();
		assert(curr != NULL);

		if (curr->fitness() < best->fitness()) {
			best->copy(curr); 
			break;
		}

		delete curr;
	}

	return best;
}

void LocalSearch::improveOperationsPositioning(Solution* S)
{
	vector<int> opids;
	double* ST = new double[S->nop()];
	for (int i = 0; i < S->nop(); ++i) if(S->isopinroute(S->getop(i) )){
		opids.push_back(i);
		ST[i] = S->st(S->getop(i));
	}
	sort(opids.begin(), opids.end(), customcomp(ST));
	reverse(opids.begin(), opids.end());
	delete[] ST;
	bool stop = true;
	do {
		stop = true;
		for (auto i : opids) stop=stop && !fwdImprovementPositioning(S, i);
		break;
	} while (!stop);
}

void LocalSearch::perturbation(Solution* S, double p)
{
	assert(p > 0 && p <= 1);

	for (int g = 0; g < _I->ngroups(); ++g) {
		double r = RAND01();
		if (r < p) {
			operation* o = S->firstgroupop(g);
			while (o != NULL) {
				S->removefromroute(o);
				o = S->nextgroupop(o);
			}
		}
	}

	_RepairHeur->solve(S);
	Solution* pS = _RepairHeur->getSolution();
	S->copy(pS);
	delete(pS);

}

LocalSearch::LocalSearch(Solver* RepairHeuristic) : Solver(RepairHeuristic->instance(),"LocalSearch")
{
	_S = NULL;
	_RepairHeur = RepairHeuristic;
}

LocalSearch::~LocalSearch()
{
	if(_S!=NULL) delete _S;
	delete _RepairHeur;
}

Solution* LocalSearch::getSolution()
{
	return new Solution(_S);
}
