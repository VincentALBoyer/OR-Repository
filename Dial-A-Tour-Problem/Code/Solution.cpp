#include "Solution.h"

string Solution::tostring(operation* o)
{
	string s;
	ostringstream prefix;
	prefix << std::left << std::setfill(' ') << setw(8) << "t=" + to_string(o->_st) << setw(8) << " c=" + to_string(o->_vcap);
	s = "["+prefix.str()+"] ";
	if (ispickupop(o)) s += "PickUp ";
	else if (isdeliveryop(o)) s += "Deliver ";
	else s = "Unknown";
	s += "group " + to_string(_I->groupid(o->_id)) + " at node " + _I->spotname(o->_id);
	s += " (op " + to_string(o->_id) + ")";
	return s;
}

bool Solution::propagateload()
{
	for (int i = 0; i < nop();++i) if (_operations[i]->_prevvehicleop == NULL) {
		operation* o = _operations[i];
		if (o->_nextvehicleop == NULL) {
			o->_vcap = 0;
		}
		else {
			o->_vcap = _I->groupsize(_I->groupid(o->_id));
			while (o->_nextvehicleop != NULL) {
				operation* nexto = o->_nextvehicleop;
				assert(nexto->_prevvehicleop == o);
				int cap = o->_vcap;
				int gsize = _I->groupsize(_I->groupid(o->_nextvehicleop->_id));
				if (ispickupop(nexto))  nexto->_vcap = o->_vcap + gsize;
				else  nexto->_vcap = o->_vcap - gsize;

				if (nexto->_vcap<0 || nexto->_vcap>_I->vcapacity())
					return false;

				o = nexto;

			}
		}
	}

	return true;

}

void Solution::initlabels()
{
	for (int i = 0; i < nop(); ++i) {
		operation* o = _operations[i];
		o->_flag = true;
		o->_st = 0;
	}
}


bool Solution::propagatelabel(operation* o)
{
	if (!o->_flag) return false;

	bool propagated = false;
	o->_flag = false;

	//next group op
	operation* nextgo = o->_nextgroupop;
	if (nextgo != NULL) {
		int st = o->_st + _I->travellingtime(o->_id, nextgo->_id);
		if (nextgo->_st < st) {
			nextgo->_flag = true;
			nextgo->_st = st;
			propagated = true;
		}
	}

	//next vehicle op
	operation* nextvo = o->_nextvehicleop;
	if (nextvo != NULL && nextvo != nextgo) {
		int st = o->_st + _I->travellingtime(o->_id, nextvo->_id);
		if (nextvo->_st < st) {
			nextvo->_flag = true;
			nextvo->_st = st;
			propagated = true;
		}
	}

	//QoS
	if (o->islastgroupop()) {
		int groupid = _I->groupid(o->_id);
		operation* first = firstgroupop(groupid);
		int st = o->_st - _I->maxtourlength(groupid);
		if (first->_st < st) {
			first->_st = st;
			first->_flag = true;
			propagated = true;
		}
	}

	return propagated;
}

bool Solution::propagatelabels()
{
	vector<int> sortedid;
	for (int i = 0; i < nop(); ++i) if (_operations[i]->isfirstvehicleop()) {
		operation* o = _operations[i];
		while (o != NULL) {
			sortedid.push_back(o->_id);
			o = o->_nextvehicleop;
		}
	}
	assert(sortedid.size() == nop());

	bool updated = true;
	while (updated ) {
		updated = false;

		for(auto i : sortedid){
			operation* o = _operations[i];
			if (o->_st > _I->planhor()) {
				return false;
			}
			updated = propagatelabel(o) || updated;
		}
	}

	if (!propagateload()) {
		return false;
	}

	return true;
}

Solution::Solution(Instance* I, bool doupdate)
{
	assert(I != NULL);
	_I = I;

	//creating the operations
	_operations = new operation * [nop()];
	for (int i = 0; i < nop(); ++i)  _operations[i] = new operation(i);
	
	//creating the links
	for (int i = 0; i < nop(); ++i) {
		if(!_I->islastnode(i)){
			int nexti = _I->nextnodeid(i);
			_operations[i]->_nextgroupop = _operations[nexti];
		}
		if(!_I->isfirstnode(i)) {
			int previ = _I->prevnodeid(i);
			_operations[i]->_prevgroupop = _operations[previ];
		}
	}

	if (doupdate) assert(update(true));
}

Solution::Solution(Solution* S):Solution(S->_I, false)
{
	copy(S);
	assert(checklinks());
}

Solution::~Solution()
{
	for (int i = 0; i < nop(); ++i) delete _operations[i];
	delete[] _operations;
	_I = NULL;
}

void Solution::copy(Solution* S)
{
	assert(S != NULL);

	for (int i = 0; i < nop(); ++i) {
		_operations[i]->_st = S->_operations[i]->_st;
		_operations[i]->_vcap = S->_operations[i]->_vcap;
		_operations[i]->_stold = S->_operations[i]->_stold;
		_operations[i]->_vcapold = S->_operations[i]->_vcapold;
		_operations[i]->_flag = S->_operations[i]->_flag;

		if (S->_operations[i]->_nextvehicleop != NULL) {
			int j = S->_operations[i]->_nextvehicleop->_id;
			_operations[i]->_nextvehicleop = _operations[j];
		}
		else _operations[i]->_nextvehicleop = NULL;

		if (S->_operations[i]->_prevvehicleop != NULL) {
			int j = S->_operations[i]->_prevvehicleop->_id;
			_operations[i]->_prevvehicleop = _operations[j];
		}
		else _operations[i]->_prevvehicleop = NULL;

		if (S->_operations[i]->_prevvehicleopold != NULL) {
			int j = S->_operations[i]->_prevvehicleopold->_id;
			_operations[i]->_prevvehicleopold = _operations[j];
		}
		else _operations[i]->_prevvehicleopold = NULL;

		if (S->_operations[i]->_nextvehicleopold != NULL) {
			int j = S->_operations[i]->_nextvehicleopold->_id;
			_operations[i]->_nextvehicleopold = _operations[j];
		}
		else _operations[i]->_nextvehicleopold = NULL;

	}
	assert(checklinks());
}

void Solution::clear()
{
	for (int i = 0; i < nop(); ++i) {
		_operations[i]->_nextvehicleop = NULL;
		_operations[i]->_prevvehicleop = NULL;
		_operations[i]->_st = 0;
		_operations[i]->_vcap = 0;
		_operations[i]->_stold = 0;
		_operations[i]->_vcapold = 0;
		_operations[i]->_flag =false;
	}
}

void Solution::removefromroute(operation* o)
{
	if (o->_prevvehicleop != NULL) {
		o->_prevvehicleop->_nextvehicleop = o->_nextvehicleop;
	}
	if (o->_nextvehicleop != NULL) {
		o->_nextvehicleop->_prevvehicleop = o->_prevvehicleop;
	}
	o->_prevvehicleop = NULL;
	o->_nextvehicleop = NULL;
}

void Solution::insertafter(operation* prevo, operation* newo)
{
	assert(prevo != NULL);
	assert(newo != NULL);
	
	removefromroute(newo);

	newo->_prevvehicleop = prevo;
	newo->_nextvehicleop = prevo->_nextvehicleop;

	if (prevo->_nextvehicleop != NULL)
		prevo->_nextvehicleop->_prevvehicleop = newo;

	prevo->_nextvehicleop = newo;

	assert(newo->_nextvehicleop != newo);
	assert(newo->_prevvehicleop != newo);
}

void Solution::insertbefore(operation* nexto, operation* newo)
{
	assert(nexto != NULL);
	assert(newo != NULL);

	removefromroute(newo);

	newo->_prevvehicleop = nexto->_prevvehicleop;
	newo->_nextvehicleop = nexto;

	if (nexto->_prevvehicleop != NULL)
		nexto->_prevvehicleop->_nextvehicleop = newo;

	nexto->_prevvehicleop = newo;

	assert(newo->_nextvehicleop != newo);
	assert(newo->_prevvehicleop != newo);
}

void Solution::swap(operation* o1, operation* o2)
{
	assert(o1 != NULL && o2 != NULL);

	operation* prev2 = o2->_prevvehicleop;
	operation* next2 = o2->_nextvehicleop;
	removefromroute(o2);
	if (prev2 == o1) insertbefore(o1, o2);
	else if (next2 == o1) insertafter(o1, o2);
	else {
		removefromroute(o1);
		if (prev2 != NULL) insertafter(prev2, o1);
		else if (next2 != NULL) insertbefore(next2, o1);
	}
}

bool Solution::fwdmove(operation* o)
{
	operation* onext = o->_nextvehicleop;
	if (onext != NULL) {
		swap(o, onext);
	}
	return onext!=NULL;
}

bool Solution::bwdmove(operation* o)
{
	operation* oprev = o->_nextvehicleop;
	if (oprev != NULL) {
		swap(o, oprev);
	}
	return oprev != NULL;
}

void Solution::createvehiclelink(operation* o, operation* d)
{
	assert(o != NULL);
	assert(d != NULL);

	d->_prevvehicleop = o;
	o->_nextvehicleop = d;
}

bool Solution::insertinroute(operation* vehicleop, operation* op, int t)
{
	if (op==NULL || isopinroute(op) || vehicleop==NULL || vehicleop==op) return false;
	operation* firstvehicleop = vehicleop;
	while (prevvehicleop(firstvehicleop) != NULL) firstvehicleop = prevvehicleop(firstvehicleop);
	setst(op, t);
	assert(firstvehicleop != NULL);
	if (st(firstvehicleop) > st(op)) {
		insertbefore(firstvehicleop, op);
	}
	else {
		operation* opnext = firstvehicleop;

		if (isdeliveryop(op)) {
			//We deliver first
			while (nextvehicleop(opnext) != NULL && st(nextvehicleop(opnext)) < st(op)) {
				opnext = nextvehicleop(opnext);
			}
		}
		else {
			//We pickup last
			while (nextvehicleop(opnext) != NULL && st(nextvehicleop(opnext)) <= st(op)) {
				opnext = nextvehicleop(opnext);
			}
		}
		insertafter(opnext, op);
	}

	return true;
}


bool Solution::isFeasible(bool printerror)
{
	vector<string> errormess;
	assert(checklinks());

	//Tour sequence
	for (int g = 0; g < _I->ngroups(); ++g) {
		int tourlength = 0;
		operation* o = firstgroupop(g);
		o = o->_nextgroupop;
		while (o != NULL) {
			assert(o->_prevgroupop != NULL);
			if (o->_prevgroupop->_st > o->_st) {
				string e = tostring(o->_prevgroupop) + " visited before " + tostring(o);
				errormess.push_back(e);
			}
			else if (o->_prevgroupop->_id == o->_id) {
				int delta = o->_st - o->_prevgroupop->_st;
				int travtime = _I->travellingtime(o->_prevgroupop->_id, o->_id);
				assert(delta > 0);
				if (delta < travtime) {
					string e = tostring(o->_prevgroupop) + " to " + tostring(o) + " must last more than " + to_string(travtime);
					errormess.push_back(e);
				}
			}
			tourlength = o->_st - firstgroupop(g)->_st;

			o = o->_nextgroupop;
		}
		if (tourlength > _I->maxtourlength(g)) {
			string e = "QoS violated for group " + to_string(g) + " : tour last more than " + to_string(_I->maxtourlength(g));
			errormess.push_back(e);
		}
	}

	//Route sequence
	for (int i = 0; i < nop();++i) if (_operations[i]->isfirstvehicleop()) {
		//Travelling time and capacity
		operation* o = _operations[i];
		if (o->_nextvehicleop == NULL) {
			string e = tostring(o) + " not covered";
		}
		else {
			assert(ispickupop(o));
			operation* onext = o->_nextvehicleop;
			int cap = _I->groupsize(_I->groupid(o->_id));
			while (onext != NULL) {
				int delta = onext->_st - o->_st;
				int travellingtime = _I->travellingtime(_I->Spot(o->_id), _I->Spot(onext->_id));
				if (_I->Spot(o->_id) != _I->Spot(onext->_id)) {
					assert(travellingtime == _I->travellingtime(o->_id, onext->_id));
				}
				if (delta < travellingtime) {
					string e = tostring(o) + " to " + tostring(onext) + " traversed in less than " + to_string(travellingtime);
					errormess.push_back(e);
				}
				if (ispickupop(onext))
					cap += _I->groupsize(_I->groupid(onext->_id));
				else
					cap -= _I->groupsize(_I->groupid(onext->_id));

				if (cap > _I->vcapacity()) {
					string e = "Vehicle capacity exceeded at " + tostring(onext) + " (" + to_string(cap) + " > " + to_string(_I->vcapacity()) + ")";
					errormess.push_back(e);
				}
				
				if (cap < 0 && onext->_prevvehicleop!=NULL) {
					string e = "Vehicle capacity is negative at " + tostring(o) + " (" + to_string(cap) + " > " + to_string(_I->vcapacity()) + ")";
					errormess.push_back(e);
				}
				o = onext;
				onext = onext->_nextvehicleop;
			}
		}
	}

	//Planning horizon
	for (int i = 0; i < nop(); ++i) {
		operation* o = _operations[i];
		if (o->_st > _I->planhor()) {
			string e = "Operation " + tostring(o) + " scheduled outside of the planing horizon " + to_string(_I->planhor());
			errormess.push_back(e);
		}
	}

	propagateload();
	for (int i = 0; i < nop(); ++i) {
		operation* o = _operations[i];
		if (o->_vcap > _I->vcapacity()) {
			string e = "*Vehicle capacity exceeded at " + tostring(o) + " (" + to_string(o->_vcap) + " > " + to_string(_I->vcapacity()) + ")";
			errormess.push_back(e);
		}
		if (o->_vcap < 0) {
			string e = "*Vehicle capacity is negative at " + tostring(o) + " (" + to_string(o->_vcap) + " > " + to_string(_I->vcapacity()) + ")";
			errormess.push_back(e);
		}
	}

	if (printerror) {
		if(!errormess.empty()) print();
		for (auto e : errormess)
			cout << e << endl;
	}
	return errormess.empty();
}

void Solution::print()
{
	cout << "Fitness: " << fitness() << endl;
	int r = -1;
	for (int i = 0; i < nop(); ++i) {
		operation* o = _operations[i];
		if (o->_prevvehicleop == NULL && o->_nextvehicleop != NULL) {
			if(r<0) cout << "Routes:" << endl;
			r++;
			operation* currento = o;
			string label = "r_" + to_string(r) + ":";
			do {
				cout << setw(10) << label << tostring(currento) << endl;
				label = " ";
				currento = currento->_nextvehicleop;
			} while (currento != NULL);
		}
	}
	if (r == -1) cout << "No route to print" << endl;

	bool hasUncOp = false;
	for (int i = 0; i < nop(); ++i) {
		operation* o = _operations[i];
		if (o->_prevvehicleop == NULL && o->_nextvehicleop == NULL) {
			if (!hasUncOp) {
				cout << "Uncovered operations: ";
				hasUncOp = true;
			}
			cout << setw(10) << tostring(o)+" ";
		}	
	}
	if (hasUncOp) cout << endl;
	else cout << "All operations are covered" << endl;

	cout << "Fitness: " << fitness() << endl;
	cout << "Peek Load: " << peekLoad() << endl;
	cout << "Average Load: " << averageLoad() << endl;

}

void Solution::printGroupTrip()
{
	int* vid = new int[nop()];
	for (int i = 0; i < nop(); ++i) vid[i] = -1;
	int r = 0;
	for (int i = 0; i < nop(); ++i) {
		operation* o = _operations[i];
		if (o->_prevvehicleop == NULL && o->_nextvehicleop != NULL) {
			operation* currento = o;
			do { 
				vid[currento->_id] = r;
				currento = currento->_nextvehicleop;
			} while (currento != NULL);
			r++;
		}
	}

	int w = 15;
	int sw = 10;
	for (int g = 0; g < _I->ngroups(); ++g) {
		cout << setw(w) << "Group" + to_string(g) + ":";

		vector<int> groupnodes;
		int i = _I->firstgroupnode(g);
		groupnodes.push_back(i);
		do {
			i = _I->nextnodeid(i);
			groupnodes.push_back(i);
		} while (!_I->islastnode(i));

		cout << setw(sw) << "sp:";
		for (auto i : groupnodes) cout << setw(sw) << _I->spotid(i); cout << endl;
		cout << setw(w) << "-" << setw(sw) << "nid:";
		for (auto i : groupnodes) cout << setw(sw) << i; cout << endl;
		cout << setw(w) << "-" << setw(sw) << "v:";
		for (auto i : groupnodes) cout << setw(sw) << vid[i]; cout << endl;
		cout << setw(w) << "-" << setw(sw) << "st:";
		for (auto i : groupnodes) cout << setw(sw) << _operations[i]->_st; cout << endl;

	}

	delete[] vid;
}

void operation::insertbefore(operation* op)
{
	op->_nextvehicleop = this;
	op->_prevvehicleop = this->_prevvehicleop;
	this->_prevvehicleop = op;
}

void operation::insertafter(operation* op)
{
	op->_prevvehicleop = this;
	op->_nextvehicleop = this->_nextvehicleop;
	this->_nextvehicleop = op;
}

void operation::commit()
{
	_stold = _st;
	_vcapold = _vcap;
	_nextvehicleopold = _nextvehicleop;
	_prevvehicleopold = _prevvehicleop;
}

void operation::revert()
{
	_st = _stold;
	_vcap = _vcapold;
	_nextvehicleop = _nextvehicleopold;
	_prevvehicleop = _prevvehicleopold;
}

bool operation::hasChanged()
{
	return (_st != _stold || _vcap != _vcapold || _nextvehicleop != _nextvehicleopold || _prevvehicleop != _prevvehicleopold);
}

bool operation::checklinks()
{
	if (_nextvehicleop != NULL && _nextvehicleop->_prevvehicleop != this) {
		return false;
	}
	if (_prevvehicleop != NULL && _prevvehicleop->_nextvehicleop != this) {
		return false;
	}
	if (_nextvehicleopold != NULL && _nextvehicleopold->_prevvehicleopold != this) {
		return false;
	}
	if (_prevvehicleopold != NULL && _prevvehicleopold->_nextvehicleopold != this) {
		return false;
	}
	return true;
}

operation::operation(int nodeid)
{	
	_id = nodeid;
	_nextgroupop = NULL;
	_prevgroupop = NULL;
	_nextvehicleop = _nextvehicleopold = NULL;
	_prevvehicleop = _prevvehicleopold = NULL;
	_st = _stold = 0;
	_vcap = _vcapold = 0;
	_flag = false;
}

operation::~operation()
{
	_nextgroupop = NULL;
	_prevgroupop = NULL;
	_nextvehicleop = NULL;
	_prevvehicleop = NULL;
}


bool Solution::update(bool docommit)
{
	assert(checklinks());
	initlabels();

	if (!propagatelabels()) {
		return false;
	}
	else {
		if (docommit) commit();
		return true;
	}
}


void Solution::commit() {
	for (int i = 0; i < nop(); ++i) _operations[i]->commit();
}

void Solution::revert() {
	for (int i = 0; i < nop(); ++i) _operations[i]->revert();
}

int Solution::fitness() {
	int fitness = 0;
	for (int i = 0; i < nop(); ++i) if (_operations[i]->isfirstvehicleop())
		fitness++;
	return fitness;
}

int Solution::ctminGTour(operation* o)
{
	if (o == NULL) return 0;
	operation* nexto = o;
	while (nexto->_nextgroupop != NULL) {
		nexto = nexto->_nextgroupop;
	}
	return nexto->_st;
}

int Solution::ctminVehicle(operation* o)
{
	if (o == NULL) return 0;
	operation* nexto = o;
	while (nexto->_nextvehicleop != NULL) {
		nexto = nexto->_nextvehicleop;
	}
	return nexto->_st;
}

bool Solution::advancedBestInsertion(operation* pickup)
{
	assert(ispickupop(pickup));
	operation* delivery = pickup->_nextgroupop;
	assert(!isopinroute(pickup));
	assert(!isopinroute(delivery));

	int groupsize = _I->groupsize(_I->groupid(pickup->_id));
	
	vector<int> Candidates;
	for (int i = 0; i < nop(); ++i) {
		operation* o = getop(i);
		bool keep = isopinroute(o);
		keep = keep && (o->_vcap + groupsize <= _I->vcapacity());
		int ct = ctminVehicle(o);
		int t_op= _I->travellingtime(o->_id, pickup->_id);
		int t_pd= _I->travellingtime(pickup->_id, delivery->_id);
		int t_dn = 0;
		int t_on = 0;
		if (o->_nextvehicleop != NULL) {
			t_dn = _I->travellingtime(delivery->_id, o->_nextvehicleop->_id);
			t_on = _I->travellingtime(o->_id, o->_nextvehicleop->_id);;
		}
		keep = keep && (ct - t_on + t_op + t_pd + t_dn <= _I->planhor());
		if (o->_nextvehicleop != NULL) {
			keep = keep && (vcap(o->_nextvehicleop) + groupsize > _I->vcapacity() || _I->travellingtime(o->_nextvehicleop->_id, pickup->_id) != 0);
			keep = keep && (st(o->_nextvehicleop) + _I->travellingtime(o->_nextvehicleop->_id, pickup->_id) > st(pickup));
		}
		
		if(keep) Candidates.push_back(i);
	}
	

	int bestdelta = INT_MAX;
	operation* besto = NULL;
	for(auto i : Candidates){
		operation* o = getop(i);
		

		insertafter(o, pickup);
		insertafter(pickup, delivery);
		if (update(false)) {
			int d = 0;
			for (int i = 0; i < nop(); ++i) d += deltast(getop(i));

			if (d < bestdelta) {
				bestdelta = d;
				besto = o;
			}

			revert();
		}
		
		if (bestdelta == 0) break;
	}

	if (besto != NULL) {
		insertafter(besto, pickup);
		insertafter(pickup, delivery);
		assert(update());
	}
	//else revert();

	return (besto!=NULL);
}

bool Solution::checklinks()
{
	for (int i = 0; i < nop(); ++i) if(!_operations[i]->checklinks()){
		return false;
	}

	return true;
}

int Solution::peekLoad()
{
	assert(propagateload());
	int vcap = 0;
	for (int i = 0; i < nop(); ++i) vcap = fmax(vcap, _operations[i]->_vcap);
	return vcap;
}

double Solution::averageLoad()
{
	assert(propagateload());
	double length = 0;
	double wsumvcap = 0;

	for (int i = 0; i < nop(); ++i) if (_operations[i]->_prevvehicleop == NULL) {
		operation* o = getop(i);
		int ST = st(o);
		while (nextvehicleop(o) != NULL) {
			o = nextvehicleop(o);
			double l = st(o) - ST;
			ST = st(o);
			length += l;
			wsumvcap += l * o->_vcap;
		}

	}

	return ceil(wsumvcap/length*100)/100.0;
}

double Solution::QoS(int gid) {
	if (gid >= 0 && gid < _I->ngroups()) {
		double tourlength = 0.0;
		operation* o = firstgroupop(gid);
		o = o->_nextgroupop;
		while (o != NULL) {
			assert(o->_prevgroupop != NULL);
			tourlength = o->_st - firstgroupop(gid)->_st;

			o = o->_nextgroupop;
		}

		return (double)_I->mintourlength(gid) / tourlength;
	}
	else {
		double Q = INFINITY;
		for (int g = 0; g < _I->ngroups(); ++g) {
			double q = QoS(g);
			Q = fmin(Q, q);
		}

		return Q;
	}
	
}

double Solution::AvQoS() {
	double AvQ = 0.0;
	for (int g = 0; g < _I->ngroups(); ++g) {
		AvQ += QoS(g);
	}

	return AvQ/(double)_I->ngroups();
}

int Solution::NOpVehicle(operation* o) {
	assert(isopinroute(o));

	while (!o->isfirstvehicleop()) o = o->_prevvehicleop;

	int N = 0;
	while (o != NULL) {
		N++;
		o = o->_nextvehicleop;
	}

	return N;
}

int Solution::MinNopVehicles() {
	int MinN = nop();
	for (int i = 0; i < nop(); ++i) if (_operations[i]->isfirstvehicleop()) {
		operation* o = _operations[i];
		MinN = fmin(MinN, NOpVehicle(o));
	}
	return MinN;
}

int Solution::MaxNopVehicles() {
	int MaxN = 0;
	for (int i = 0; i < nop(); ++i) if (_operations[i]->isfirstvehicleop()) {
		operation* o = _operations[i];
		MaxN = fmax(MaxN, NOpVehicle(o));
	}
	return MaxN;
}
