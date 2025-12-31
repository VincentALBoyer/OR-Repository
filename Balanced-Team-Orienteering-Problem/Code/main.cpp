#include "Solver.h"
#include "MILPSolver.h"
#include "BandC.h"

int main(int argc, char* argv[])
{
    assert(argc > 1);
    string filename = argv[1];
    double timlim = 3600;
    if (argc >= 3) timlim = stod(string(argv[2]));

    Instance *I=new Instance(filename);   

    cout << I->instancename() << endl;
 
    Solver* S = new BandC(I);

    S->setparam(Solver::TimLim, timlim);
    S->setparam(Solver::Threads, 1);
    S->solve();
    S->save();

    delete S;
    delete I;
}
