#pragma once

#include "Header.h"

enum class ParamCompObject{ntours,minnodetour,maxnodetour, mintrlength, maxtrlength,nhotels,ngroups,vcapacity,mingsize,maxgsize,maxd,mind,planhor,seed,QoS, minl, maxl,timlim,instance,datafile,folderpath,help,gendata, geninst, solve, solver, initsolver, avrep};
enum class SolverAlgo {Greedy2, MILP3, CP1, CP2, LS, Unknown, None };

class Parameter;

class ParameterComponent {
    friend class Parameter;

    ParamCompObject _pco;
    string _val;            //value of the parameter if string
    string _name;
    string _description;
    string _d_val;          //Default Value
    bool _isUserDefined;

    void setVal(string val, bool UsDef = true) {
        _val = val;
        _isUserDefined = UsDef;
    }

    bool isValInteger()
    {
        char* p;
        strtol(_val.c_str(), &p, 10);
        return *p == 0 && _val != "";
    }

    string Value() { return _val; }
    int IntValue() {
        assert(isValInteger());
        return atoi(_val.c_str());
    }

    bool isHelp() { return _pco == ParamCompObject::help; }

    int extractdata(int argc, char* argv[]);

    string tostring();


public:
    ParameterComponent(ParamCompObject pco, string Name, string Value, string Description="");
  
};

class Parameter : private vector<ParameterComponent*>{

    vector<string> _ErrorMessage;

    ParameterComponent* getParamComponent(ParamCompObject P);

    bool checkFile(string filepath);

    string getParamName(ParamCompObject P);

    void validateIsLowerThan(ParamCompObject L, ParamCompObject R);

    void validateIsStrictlyLowerThan(ParamCompObject L, ParamCompObject R);

    void validateParametersList(int argc, char* argv[]);

    void validateRange(ParamCompObject L, int min=1, int max=INT_MAX);

    void validateFile(ParamCompObject L);

    void checkCompatibility(ParamCompObject P, ParamCompObject Q) {
        if (P!=Q && isUserDefined(P) && isUserDefined(Q)) {
            ostringstream Error;
            Error << "Parameter -" << getParamComponent(P)->_name << " not compatible with parameter -" << getParamComponent(Q)->_name;
            _ErrorMessage.push_back(Error.str());
        }
    }
    SolverAlgo SolverMapping(string Solver) {

        if (Solver == tostring(SolverAlgo::Greedy2)) return SolverAlgo::Greedy2;
        else if (Solver == "MILP3") return SolverAlgo::MILP3;
        else if (Solver == "CP1") return SolverAlgo::CP1;
        else if (Solver == "CP2") return SolverAlgo::CP2;
        else if (Solver == "LS") return SolverAlgo::LS;
        else if (Solver == "None") return SolverAlgo::None;
        else return SolverAlgo::Unknown;
    }

public:

    static string tostring(SolverAlgo A) {
        if (A == SolverAlgo::Greedy2) return "Greedy2";
        else if (A == SolverAlgo::MILP3) return "MILP3";
        else if (A == SolverAlgo::CP1) return "CP1";
        else if (A == SolverAlgo::CP2) return "CP2";
        else if (A == SolverAlgo::CP3) return "CP3";
        else if (A == SolverAlgo::LS) return "LS";
        else if (A == SolverAlgo::None) return "None";

        return "Unknown";
    }

    Parameter(int argc, char* argv[]);

    ~Parameter();

    bool isHelpRequired();

    bool hasErrors() { return !_ErrorMessage.empty(); }

    bool hasInstance() { return isUserDefined(ParamCompObject::solve); }

    void displayErrors();

    string getParamValue(ParamCompObject P);

    int getParamIntValue(ParamCompObject P);

    bool isUserDefined(ParamCompObject P);
    
    void print();

    SolverAlgo getSelectedSolver() {
        string Solver = getParamValue(ParamCompObject::solver);
        return SolverMapping(Solver);
    }

    SolverAlgo getSelectedInitSolver() {
        string Solver = getParamValue(ParamCompObject::initsolver);
        return SolverMapping(Solver);
    }
};
