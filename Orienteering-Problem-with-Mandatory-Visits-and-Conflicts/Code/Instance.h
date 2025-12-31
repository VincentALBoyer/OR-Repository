#pragma once
#include "Header.h"

class Instance;

class Node {
    friend class Instance;
    int _id;
    double _x;
    double _y;
    double _st; // service time
    double _s; // score

public:
    static const double timefactor;
    Node();
    ~Node() {}

	bool isdepot() const { return _id == 0; } //Initial depot is node 0
    bool iscustomer() const { return !isdepot(); }
    double distanceto(Node *N);
    string tostring();
    int index() const { return _id; }
    double score() const { return _s; };
    double serviceTime() { return /*ceil*/(_st*timefactor); };
};


class Instance {

    

    int _nnodes;        //Number of nodes
    Node **_nodes;      //Array of nodes
    double _tmax;          //time limit
    int _nvehicles;     //Number of vehicles
    string _name;       //Instance Name
    list<int>* _comp;   // compatibilities
    double **_dist;		// distances
    string _type;			// type of file

    unordered_set<int>* _fastcomp;

	vector<int> _reachablenodes; // reachable nodes for each vehicle
	vector< set<int> > _conflictnodesets; // conflict node pairs 

	void initcompatibilities(int size);

	void transformtosymmetricdistances(); // transform asymmetric distances to symmetric distances

public:
	//unordered_setInstance(string filename);
	Instance(string filename, string);
	~Instance();

    const vector<int>& reachablenodes() const { return _reachablenodes; }
	const vector< set<int> >& conflictnodesets() const { return _conflictnodesets; }

    int nvehicles() const { return _nvehicles; }
    int nnodes() const { return _nnodes; }
    string name() { return _name; }
    Node* node(int i) {return _nodes[i];}
    Node* depot() {return _nodes[0];}
    double distance(Node*, Node*);
    bool comp(int k, int i);
    bool fastcomp(int k, int i);
    double tmax() { return /*floor*/(_tmax*Node::timefactor); }
    double totalScore();
	void print();
	void print_costs(string );
	//int get_distance(int i, int j) { return _dist[i][j]; };
	list<int> compRoute(int k) { return _comp[k]; };
	double score_comp(int r);
	string getType() { return _type; }
};
