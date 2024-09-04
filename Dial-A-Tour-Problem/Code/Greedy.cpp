#include "Greedy.h"

void Greedy::solvemethod()
{
	if(_A == GreedyAlgo::Greedy2) solvemethod2();
	else {
		cout << "Error: Selected Algorithm is Unknown for Greedy Solver" << endl;
		exit(222);
	}
}

void Greedy::solvemethod2()
{
	vector<int> Candidates;
	double* pt = new double[_I->ngroups()];
	for (int g = 0; g < _I->ngroups(); ++g) {
		Candidates.push_back(g);
		pt[g] = _I->mintourlength(g);
	}
	sort(Candidates.begin(), Candidates.end(), customcomp(pt));
	reverse(Candidates.begin(), Candidates.end());

	int et = 0;
	operation* lastvehicleop = NULL;
	while (!Candidates.empty()) {

		auto it = Candidates.begin();
		int g = -1;
		int dist = 0;
		while (it != Candidates.end()) {
			g = *it;
			if (lastvehicleop!=NULL) {
				int i = _S->getid(lastvehicleop);
				int j = _I->firstgroupnode(g);
				dist = _I->travellingtime(i, j);
			}

			if (et + dist + _I->mintourlength(g) <= _I->planhor()) break;

			it++;
		}

		if (it == Candidates.end()) {
			et = 0;
			lastvehicleop = NULL;
		}
		else {
			assert(g == *it);
			Candidates.erase(it);
			et += dist + _I->mintourlength(g);
			assert(et <= _I->planhor());

			//We try to merge with other groups
			vector<int> G;
			G.push_back(g);
			int load = _I->groupsize(g);
			auto itmerge = Candidates.begin();
			while (itmerge != Candidates.end()) {
				int gg = *itmerge;
				int i = _I->firstgroupnode(g);
				int j = _I->firstgroupnode(gg);
				dist = _I->travellingtime(i, j);
				int grsize = _I->groupsize(gg);
				if (dist <= 0.001 && load + grsize <= _I->vcapacity() && _I->tourid(j) == _I->tourid(i)) {
					G.push_back(gg);
					itmerge = Candidates.erase(itmerge);
					load += grsize;
				}
				else itmerge++;
			}

			//We insert in parallel all group in G
			operation** grop = new operation*[G.size()];
			int k = 0;
			for (auto gg : G) grop[k++] = _S->firstgroupop(gg);
			assert(k == G.size());
			while (grop[0] != NULL) {
				for (int k = 0; k < G.size();++k) {
					if (lastvehicleop != NULL)  _S->insertafter(lastvehicleop, grop[k]);
					
					lastvehicleop = grop[k];

					grop[k] = _S->nextgroupop(grop[k]);
				}
			}

			assert(_S->update(false));

			assert(et == _S->st(lastvehicleop));

			delete[] grop;
		}
	}
}

void Greedy::initsolver(Solution* S)
{
	if (S != NULL) {
		_S->copy(S);
	}
	else _S->clear();
}

Greedy::Greedy(Instance* I, GreedyAlgo A) : Solver(I, Parameter::tostring(A))
{
	_alpha = 0.9;
	_S = new Solution(_I);
	_S->update();

	_A = A;
}

Greedy::~Greedy()
{
	delete _S;
}

Solution* Greedy::getSolution()
{
	return new Solution(_S);
}
