#include "Solution.h"

Solution::Solution(Instance* I) : vector<int>()
{
    _I=I;
}

Solution::~Solution()
{

}

double Solution::fitness(){
    double F=0;
    for(auto i : *this){
        node* N=_I->Node(i);
            F+=N->score();
        
    }
    
    return F;
}

bool Solution::isFeasible()
{
    vector<string> ErrorMessages;
	
    //Mandatory Nodes
    for(int i=0 ; i<_I->nnodes();++i) if(_I->Node(i)->isMandatoryNode()){
        assert(i==_I->Node(i)->id());
        
        if(find(this->begin(),this->end(),i)==this->end()){
            ErrorMessages.push_back("Error: Mandatory Node "+to_string(i) +" Not Visited");
        }
    }
    
    //Conflicts
    for(auto i : *this){
        auto C=_I->Node(i)->getConflictList();
        for(auto Nj : C){
            int j=Nj->id();
            if(find(this->begin(),this->end(),j)!=this->end()){
                ErrorMessages.push_back("Error: Conflict between Node "+to_string(i) +" and Node "+to_string(j));
            }
        }
    }
    
    //Time Max
    double T=0;
    node* Ni=_I->InitialDepot();
    for(auto j : *this) {
        node* Nj=_I->Node(j);
        if (Ni == Nj) continue;
        T+=_I->travellingtime(Ni, Nj);
        Ni=Nj;
    }
    T+=_I->travellingtime(Ni, _I->FinalDepot());
    
    if(T>_I->Tmax())
        ErrorMessages.push_back("Error: Duration Max Exceeded: "+to_string(T) +" > "+to_string(_I->Tmax()));
    
    for(auto E : ErrorMessages){
        cout<<E<<endl;
    }
    
    
	return ErrorMessages.empty();
}


void Solution::print(){
    cout<<"Fitness: "<<fitness()<<endl;
    cout<<"Route: ";
    for(auto i : *this) cout<<i<<" ";
    cout<<endl;
    
    
    
}
