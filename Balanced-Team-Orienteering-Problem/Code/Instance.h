#pragma once
#include "Header.h"


enum class DistanceMode {Euclidian=0, Pseudoeuclidian=1, Fischetti=2, Geographical=3};

enum class NodeType {InitialDepot=0, FinalDepot=1, OptionalNode=2, MandatoryNode=3, InfeasibleNode=4};

class Instance;

class node{
    int _id;
    double _x;
    double _y;
    int _s;
    
    NodeType _type;
    
    vector<node* > _Conflicts;
    
    friend class Instance;
    friend class HeuristicSolver;
    friend class ImprovingHeuristicSolver;
    
public:

    node(int i=0, double x=0, double y=0, int s=0, NodeType T=NodeType::OptionalNode);
    ~node() {};
    
    int id() {return _id;};
    double x() const {return _x;};
    double y() const {return _y;};
    int score() {return _s;};
    
    bool isFeasible() { return _type != NodeType::InfeasibleNode; }
    void setInfeasible() { _type = NodeType::InfeasibleNode; }
    bool isInitialDepot() {return _type==NodeType::InitialDepot;};
    bool isFinalDepot() {return _type==NodeType::FinalDepot;};
    bool isDepot() { return isInitialDepot() || isFinalDepot(); }
    bool isMandatoryNode() {return _type==NodeType::MandatoryNode;};
    bool isOptionalNode() {return _type==NodeType::OptionalNode;};
    vector<node*> getConflictList() {return _Conflicts;}
    bool hasConflictWith(node* N) { return find(_Conflicts.begin(), _Conflicts.end(), N) != _Conflicts.end(); }
    
    void print();
    
};

class Instance {
    
    string _filename;

    int _Tmax;
    
    DistanceMode _DM;
    
    int _ndepots;
    
    int _nnodes;
    
    int _InitialDepotId;
    
    int _FinalDepotId;
    
    node** _Nodes;

    double coordtogeo(double val) {
        double pi = 3.141592;
        double deg = (int)val;
        double min = val - deg;
        return pi * (deg + 5.0 * min / 3.0) / 180;
    }

public:
	Instance(string filename);
	~Instance();
    
    string instancename() {
        size_t pos = _filename.find_last_of("/");
        return _filename.substr(pos + 1);
        
    }

    int Tmax() {return _Tmax;}
    int nnodes() {return _nnodes;}
    
    node* InitialDepot() {return _Nodes[_InitialDepotId];};
    node* FinalDepot() {return _Nodes[_FinalDepotId];};
    node* Node(int i) {return _Nodes[i];}
    
    double travellingtime(node* Ni, node* Nj);
    
    bool areCompatible(node* I, node* J) {
        assert(I != NULL && J != NULL);
        return !I->hasConflictWith(J) && !J->hasConflictWith(I);
    }
    
	void print();

};

