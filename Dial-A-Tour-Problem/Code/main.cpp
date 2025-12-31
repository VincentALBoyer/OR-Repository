#include "Instance.h"
#include "Solution.h"
#include "CPSolver.h"
#include "MILPSolver.h"
#include "Greedy.h"
#include "LocalSearch.h"

Solver* getSolver(SolverAlgo A, Instance* I) {
    if (A == SolverAlgo::MILP3) return new MILPSolver(I, A);
    else if (A == SolverAlgo::CP1 || A == SolverAlgo::CP2) return new CPSolver(I, A);
    else if (A == SolverAlgo::Greedy2) return new Greedy(I,A);
    else if (A == SolverAlgo::LS ) return new LocalSearch(new Greedy(I,SolverAlgo::Greedy2));
    else return NULL;
}

Solver* getSolver(Parameter* P,Instance* I) {
    Solver* S = NULL;

    Solver* S1 = getSolver(P->getSelectedInitSolver(),I);
    Solver* S2 = getSolver(P->getSelectedSolver(), I);

    if (S2 == NULL) {
        if (S1 != NULL) delete S1;
    }
    else if (S1 == NULL) {
        if (P->getSelectedInitSolver() == SolverAlgo::Unknown) {
            if (S2 != NULL) delete S2;
        }
        else S = S2;
    }
    return S;
}

int main(int argc, char* argv[])
{
    Parameter* P = new Parameter(argc, argv);
    P->print();

    if (P->isHelpRequired()) {
        exit(10);
    }
    else if (P->hasErrors()) {
        P->displayErrors();
        exit(20);
    }
    
    Instance* I = new Instance(P);

    Solver* H = getSolver(P,I);
    if (H == NULL) {
        cout << "Solver not found... exiting" << endl;
        exit(40);
    }

    H->solve();

    H->save("Output/Results_" + H->name() + "_" + to_string(I->qos())+"_"+to_string(P->getParamIntValue(ParamCompObject::timlim)) + ".txt");

    Solution* S = H->getSolution(); 

    if (S != NULL) {
        S->print();
        assert(S->isFeasible(true));
    }
    else cout << "No Feasible Solution Found" << endl;

    delete H;
    delete P;
    if (I != NULL) delete I;
}
