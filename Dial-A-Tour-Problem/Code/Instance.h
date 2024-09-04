#pragma once

#include "parameter.h"

class TwoDimNode {
	double _x, _y, _l;
	static double _corr;
public:
	TwoDimNode(double x, double y, double l) {
		_x = x;
		_y = y;
		_l = l;
	}
	TwoDimNode(const vector<TwoDimNode*>& Points, double mindist, double l = 60);
	double x() { return _x; }
	double y() { return _y; }
	double l() { return _l; }

	bool isPresent(const vector<TwoDimNode*>& Points) {
		for (auto P : Points) if (this == P) return true;
		return false;
	}

	double distanceto(const TwoDimNode& P) {
		return ceil(_corr * sqrt(pow(_x - P._x, 2) + pow(_y - P._y, 2)));
	}

	static void setcorr(double val) { _corr = val; }

	static double length(const vector<TwoDimNode*>& Points);

	static void print(const vector<TwoDimNode*>& Points);
};

class Instance;
class node;
enum class NodeType {pickup, delivery};
string to_string(NodeType T);

class spot {
	friend class Instance;
	friend class node;
	static int _idgen;
	int _id;				//for internal use
	int _spotid;			//id of the spot to pickup/deliver the group
	string _spotname;		//name given to the spot
	int _lengthofstay;

	int _a, _b;				//time windows for future extension

public:
	spot(int spotid, int lengthofstay, int a = 0, int b = INT_MAX, string spotname = "");
};

class tour : public vector<spot*>{
	friend class Instance;
	friend class node;
	static int _idgen;
	int _id;				//for internal use
	int _trid;				//tour id this node is part
	string _trname;			//name given to the trip

public:
	tour(int trid, string tourname = "");
};

class group {
	friend class Instance;
	friend class node;
	static int _idgen;
	int _id;				//for internal use
	int _grid;
	tour* _tour;			//assined tour
	int _grsize;			//group size
	spot* _origin;			//hotel/spot of origin
	int _maxtrlength;		//maximum time required to perform the tour from QoS
	int _mintrlength;		//minimum time required to complete the tour
	node* _firstgroupnode;	//first pickup node

public:
	group(int grid, int grsize, tour* tour, spot* origin);
};

class node {
	friend class Instance;
	static int _idgen;
	int _id;				//for internal use
	spot* _spot;			//the spot
	group* _group;			//the group
	NodeType _type;			//pickup or delivery

	node* _prev, * _next;	//previous and next node in the tour
	
	string extendedname();

public:
	node(group *group, spot* spot, NodeType type);
	~node() {}
};

class Instance
{
	Parameter* _Param;

	int _nspots;		//The number of real spots in the problem
	int _nnodes;		//The number of nodes after duplication per group and pickup/delivery type
	int _ngroups;
	int _ntours;
	int _vcapacity;
	int _planhor;

	node** _nodes;				//pickup/delivery nodes
	spot** _spots;				//spots data
	group** _groups;			//groups data
	tour** _tours;				//tours data

	int** _timematrix;

	int _LowerBound;			//A Lower Bound of the Instance
	
	void resetallidgen();

	static void skipFileComments(ifstream* input);

	void initnodes();

	void read();

	void correctTimeMatrix();

	string tostring(node *N);

	string tostring(group* G);

	string tostring(tour* T);

public:

	Instance(Parameter *Param);

	~Instance();

	void save(string filename="");

	string instancename();

	int nnodes() { return _nnodes; }
	int nspots() { return _nspots; }
	int ngroups() { return _ngroups; }
	int ntours() { return _ntours; }
	int vcapacity() { return _vcapacity; }
	int planhor() { return _planhor; }
	int qos() { return _Param->getParamIntValue(ParamCompObject::QoS); }

	int firstgroupnode(int groupid) { return _groups[groupid]->_firstgroupnode->_id; }
	int groupsize(int groupid) { return _groups[groupid]->_grsize; }

	int groupid(int nodeid) { return _nodes[nodeid]->_group->_id; }
	bool islastnode(int nodeid) { return _nodes[nodeid]->_next == NULL; }
	bool isfirstnode(int nodeid) { return _nodes[nodeid]->_prev == NULL; }
	int nextnodeid(int nodeid) { 
		assert(!islastnode(nodeid));
		return _nodes[nodeid]->_next->_id; 
	}
	int prevnodeid(int nodeid) {
		assert(!isfirstnode(nodeid));
		return _nodes[nodeid]->_prev->_id;
	}
	int tourid(int nodeid) { return _nodes[nodeid]->_group->_tour->_id; }
	string tourname(int nodeid) { return _nodes[nodeid]->_group->_tour->_trname; }

	string spotname(int nodeid) { return _nodes[nodeid]->_spot->_spotname; }
	int a(int nodeid) { return _nodes[nodeid]->_spot->_a; }
	int b(int nodeid) { return _nodes[nodeid]->_spot->_a; }
	spot* Spot(int nodeid) { return _nodes[nodeid]->_spot; }
	spot* getspot(int spotid) { return _spots[spotid]; }
	int spotid(int nodeid) { return _nodes[nodeid]->_spot->_id; }
	int lengthofstay(spot* S) { return S->_lengthofstay; }

	bool ispickupnode(int nodeid) { return _nodes[nodeid]->_type == NodeType::pickup; }
	bool isdeliverynode(int nodeid) { return _nodes[nodeid]->_type == NodeType::delivery; }
	NodeType nodetype(int nodeid) { return _nodes[nodeid]->_type; }

	int travellingtime(int o_nodeid, int d_nodeid);
	int travellingtime(spot* O, spot* D);

	int maxtourlength(int groupid) { return _groups[groupid]->_maxtrlength; }
	int mintourlength(int groupid) { return _groups[groupid]->_mintrlength; }

	void print();

	int TimLim() { return _Param->getParamIntValue(ParamCompObject::timlim); }

	void setLB(int val) { _LowerBound = fmax(val,_LowerBound); }
	int LB() { return _LowerBound; }
};
