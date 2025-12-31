// OPVC Main Entry Point

#include "Solver.h"
#include "MILPSolver.h"
#include "CPSolver.h"
#include "LazyMILP.h"

int main(int argc, char* argv[])
{
    assert(argc > 1);
    // default configuration
    string filename = argv[1];
    string fileType = "implicit";
    double timlim = 10;
    int OF = 1; // objective function
    string solfile;

    if (argc >= 3) {
    	fileType = argv[2];
        assert(fileType == "implicit" || fileType == "explicit");
    	OF = stoi(string(argv[3]));
    	timlim = stod(string(argv[4]));
    }

    Instance* I = new Instance(filename, fileType);
 
    I->print();

    // Solve with Lazy MILP algorithm
    Solver *S1 = new LazyMILP(I, OF);
	S1->setparam(Solver::TimLim, timlim);
    S1->setparam(Solver::Threads, 4);
	S1->solve(NULL);
	Solution *Sol1 = S1->recoversolution();
    
    Sol1->printSolution();
    assert(Sol1->isFeasible());
	S1->saveStat(Sol1, OF);
	Sol1->saveSolution(S1->value(), OF);


    delete S1;
    delete Sol1;
    delete I;

	cout << "Finished process. Have a nice day :) " << endl;
}
