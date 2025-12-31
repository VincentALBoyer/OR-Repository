#include "Instance.h"


node::node(int i, double x, double y, int s, NodeType T){
    _id=i;
    _x=x;
    _y=y;
    _s=s;
    _type=T;
}

void node::print(){
    string T="N_";
    if(isInitialDepot()) T="O_";
    else if(isFinalDepot()) T="D_";
    else if(isMandatoryNode()) T="N*_";
    else if (!isFeasible()) T = "X_";
    cout<<T<<_id<<"("<<_x<<", "<<_y<<", "<<_s<<")";
    if(_Conflicts.empty()) cout<<"Empty";
    else{
        for(auto N : _Conflicts) cout<<N->_id<<" ";
    }
    cout<<">"<<endl;
    
}

Instance::Instance(string filename)
{
    _filename=filename;
    
    ifstream input(filename);
    assert(input.is_open());
    
    int d=0;
    input>>_Tmax >> d >> _ndepots;
    
    _DM=(DistanceMode)d;
    
    auto pos=input.tellg();
    
    //Getting Basic Nodes Info
    vector<node*> Nodes;
    int count=0;
    do{
        node* N=new node();
        int t=0;
        input>>N->_id>>N->_x>>N->_y>>N->_s>>t;
        if(t==1) N->_type=NodeType::MandatoryNode;

        if(N->id()==0) N->_type=NodeType::InitialDepot;
        if(_ndepots==2 && N->id()==1) N->_type=NodeType::FinalDepot;
        
        
        assert(N->_id==count);
        count++;
        Nodes.push_back(N);
        
        input.ignore(numeric_limits<streamsize>::max(),'\n');
        
    }while(input.peek()!=EOF && input.peek()!='\n');
    
    //Getting depots Ids
    _InitialDepotId=0;
    if(_ndepots==1){
        node* O=Nodes.front();
        assert(O->_id==0);
        node* N=new node((int)Nodes.size(),O->_x,O->_y,O->_s,NodeType::FinalDepot);
        Nodes.push_back(N);
        _FinalDepotId=N->_id;
    }
    else _FinalDepotId=1;
    
    

    //Init nodes array
    _nnodes=(int)Nodes.size();
    _Nodes=new node*[_nnodes];
    for(auto N : Nodes) _Nodes[N->_id]=N;
    
    //Getting the conflicts
    input.clear();
    input.seekg(pos);
    int i=0;
    do{
        node* N=_Nodes[i++];
        int id;
        string buffer;
        input>>id>>buffer>>buffer>>buffer>>buffer;
        assert(N->_id==id);
        
        while (input.peek() == ' ' || input.peek() == '\t') input.ignore(1, ' ');
        

        while(input.peek()!='\n'){
            int cid;
            input>>cid;
            assert(_Nodes[cid]->_id==cid);
            N->_Conflicts.push_back(_Nodes[cid]);
            if (_Nodes[cid]->isMandatoryNode())
                N->setInfeasible();
            while (input.peek() == ' ' || input.peek() == '\t') input.ignore(1, ' ');
        }
        
        input.ignore(numeric_limits<streamsize>::max(),'\n');
        
    }while(input.peek()!=EOF && input.peek()!='\n');



    // Checking for infeasible nodes 
    node* S = _Nodes[_InitialDepotId];
    node* T = _Nodes[_FinalDepotId];
    for (auto N : Nodes) {
        for (int i = 0; i < Nodes.size(); i++) {
            node* I = Nodes[i];
            if (I == N) continue;

            for (int j = 0; j < i; j++) {
                node* J = Nodes[j];

                if (I == S || I == T || J == S || J == T) {
                    continue; 
                }

                double dij = travellingtime(I, J);
                double d1 = travellingtime(S, I) + dij + travellingtime(J, T);
                double d2 = travellingtime(S, J) + dij + travellingtime(I, T);
                             
                if (d1 > _Tmax && d2 > _Tmax) {

                    auto conflictIExists = find_if(I->_Conflicts.begin(), I->_Conflicts.end(),
                        [J](node* conflict) { return conflict == J; }) != I->_Conflicts.end();

                    auto conflictJExists =find_if(J->_Conflicts.begin(), J->_Conflicts.end(),
                        [I](node* conflict) { return conflict == I; }) != J->_Conflicts.end();                    
                  
                    if (!conflictIExists) {
                        I->_Conflicts.push_back(J);
                       
                    }
                    if (!conflictJExists) {
                        J->_Conflicts.push_back(I);
                       
                    }
            
                }

            }
        }
    }



    for (int i = 0; i < nnodes(); ++i) if (!Node(i)->isOptionalNode() && !Node(i)->isInitialDepot())
        Node(i)->_s = 0;

}


Instance::~Instance()
{
    for(int i=0;i<_nnodes;i++) delete _Nodes[i];
    delete[] _Nodes;
}

void Instance::print()
{
    cout<<"File: "<<_filename<<endl;
    cout<<"Tmax= "<<_Tmax<<" - DM= "<<(int)_DM<<" - #depots= "<<_ndepots<<endl;
    for(int i=0;i<_nnodes;i++) _Nodes[i]->print();
}

double Instance::travellingtime(node* Ni, node* Nj){
    double dij = 0;
    if (_DM == DistanceMode::Euclidian)
    {
        dij= (sqrt(pow(Ni->_x - Nj->_x, 2) + pow(Ni->_y - Nj->_y, 2)));
    }
    else if (_DM == DistanceMode::Pseudoeuclidian)
    {
        
        double rij = sqrt((pow(Ni->_x - Nj->_x, 2) + pow(Ni->_y - Nj->_y, 2))/10);
        
        dij = ceil(rij);
    }
    else if (_DM == DistanceMode::Fischetti)
    {
        double rij = 100*sqrt((pow(Ni->_x - Nj->_x, 2) + pow(Ni->_y - Nj->_y, 2)));
        dij= (rij);
        
    }
    else if (_DM == DistanceMode::Geographical)
    {
        //Convert to lat and lng
        
        double lat_i = coordtogeo(Ni->_x);
        double lng_i = coordtogeo(Ni->_y);

        double lat_j = coordtogeo(Nj->_x);
        double lng_j = coordtogeo(Nj->_y);

        double rrr = 6378.388; //Earth radius
        double q1 = cos(lng_i  - lng_j );
        double q2 = cos(lat_i  - lat_j );
        double q3 = cos(lat_i  + lat_j );
        dij = (int)(rrr * acos(0.5 * ((1.0 + q1) * q2 - (1.0 - q1) * q3)) + 1.0);
    }
    else
    {
        cout << "Error: Distance Mode is not implemented" << endl;
        exit(100);
    }
    return dij;
}
