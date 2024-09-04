#pragma once

#include "Instance.h"

class Solution;


class operation {
	friend class Solution;
	int _id;							//operation id

	operation* _nextgroupop;
	operation* _prevgroupop;
	operation* _nextvehicleop;
	operation* _prevvehicleop;

	int _st;
	int _vcap;
	bool _flag;
	
	//For commits
	int _stold;
	int _vcapold;
	operation* _nextvehicleopold;
	operation* _prevvehicleopold;

	void insertbefore(operation *op);
	void insertafter(operation* op);

	bool isfirstgroupop() { return _prevgroupop == NULL; }
	bool islastgroupop() { return _nextgroupop == NULL; }

	bool isfirstvehicleop() { return _prevvehicleop == NULL; }
	bool islastvehiclevop() { return _nextvehicleop == NULL; }

	void commit();
	void revert();

	bool hasChanged();

	bool checklinks();

public:

	operation(int nodeid);

	~operation();

};


class Solution
{
	Instance* _I;

	operation** _operations;

	bool propagateload();

	void initlabels();

	bool propagatelabel(operation* o);

	bool propagatelabels();

public:

	Solution(Instance* I, bool doupdate = true);

	Solution(Solution* S);

	~Solution();

	void copy(Solution* S);

	void clear();

	int nop() { return _I->nnodes(); }

	string tostring(operation* o);
	operation* firstgroupop(int groupid) { return _operations[_I->firstgroupnode(groupid)]; }
	operation* getop(int nodeid) { return _operations[nodeid]; }
	int getid(operation* o) { return o->_id; }
	void setst(operation* o, int val) { o->_st = val; }
	int st(operation* o) { return o->_st; }
	void setvcap(operation* o, int val) { o->_vcap = val; }
	int vcap(operation* o) { return o->_vcap; }
	operation* nextvehicleop(operation* o) { return o->_nextvehicleop; }
	operation* nextgroupop(operation* o) { return o->_nextgroupop; }
	operation* prevvehicleop(operation* o) { return o->_prevvehicleop; }
	operation* prevgroupop(operation* o) { return o->_prevgroupop; }
	bool isopinroute(operation* o) { return o->_nextvehicleop != NULL || o->_prevvehicleop != NULL; }
	bool ispickupop(operation* o) { return _I->ispickupnode(o->_id); }
	bool isdeliveryop(operation* o) { return _I->isdeliverynode(o->_id); }
	double deltast(operation* o) { return o->_st - o->_stold; }

	void removefromroute(operation* o);

	void insertafter(operation* prevo, operation* newo);
	void insertbefore(operation* nexto, operation* newo);
	void swap(operation* o1, operation* o2);
	bool fwdmove(operation* o);
	bool bwdmove(operation* o);
	void createvehiclelink(operation* o, operation* d);
	bool insertinroute(operation* vehicleop, operation* op, int t);

	bool isFeasible(bool printerror=false);

	void print();

	void printGroupTrip();

	bool update(bool docommit = true);

	void commit();

	void revert();

	int fitness();

	int ctminGTour(operation* o);

	int ctminVehicle(operation* o);

	bool advancedBestInsertion(operation* pickup);

	bool checklinks();

	int peekLoad();

	double averageLoad();

	double QoS(int gid=-1);

	double AvQoS();

	int NOpVehicle(operation* o);

	int MinNopVehicles();

	int MaxNopVehicles();

};
