#include "parameter.h"


ParameterComponent::ParameterComponent(ParamCompObject pco, string Name, string Value, string Description)
{
	_pco = pco;
	_name = Name;
	_d_val = _val = Value;
	_description = Description;
	_isUserDefined = false;
}

int ParameterComponent::extractdata(int argc, char* argv[])
{
	int paramocc = 0;
	for (int k = 1; k < argc; ++k) {
		string input(argv[k]);
		input.erase(0,1);	//Removing the "-"   
		size_t pos = input.find("=");
		string paramname=input.substr(0, pos);
		
		if (paramname == _name) {
			if (pos >= input.size() /*&& isHelp()*/) {
				paramocc++;
				setVal("");
			}
			else if (pos < input.size()) {
				paramocc++;
				setVal(input.substr(pos + 1));
			}

		}
	}
	return paramocc++;
}

string ParameterComponent::tostring() {
	int w = 20;
	ostringstream s;
	if (isHelp() || _pco==ParamCompObject::gendata || _pco == ParamCompObject::geninst || _pco == ParamCompObject::solve) {
		s << setw(w) << "-" + _name << setw(2 * w) << _description;
	}
	else {
		s << setw(w) << "-" + _name + "=" << setw(2 * w) << _description;
		if (_d_val != "" || _isUserDefined) {
			s << " (";
			//if (_d_val != "") s << "default value: " << _d_val;
			//if (_d_val != "" && _isUserDefined) s << " - ";
			if (_isUserDefined) s << "user value: " << _val;
			else s << "default value: " << _d_val;
			s << ")";
		}
	}
	//if (_d_val!="") s << "(default value: " << _d_val << ")";
	return s.str();
}

bool Parameter::isHelpRequired() {
	bool helpuser = false;
	return isUserDefined(ParamCompObject::help);
}

void Parameter::displayErrors()
{
	if (!hasErrors()) {
		cout << "No Errors detected!" << endl;
	}
	else {
		for (auto s : _ErrorMessage) {
			cout << "Error: " << s << endl;
		}
	}
}

string Parameter::getParamValue(ParamCompObject P)
{	
	ParameterComponent* PC = getParamComponent(P);
	return PC->Value();
}

int Parameter::getParamIntValue(ParamCompObject P)
{
	ParameterComponent* PC = getParamComponent(P);
	assert(PC->isValInteger());
	return PC->IntValue();
}

bool Parameter::isUserDefined(ParamCompObject P)
{
	ParameterComponent* PC = getParamComponent(P);
	return PC->_isUserDefined;
}


void Parameter::print()
{
	int w = 20;

	cout << resetiosflags(std::ios::adjustfield);
	cout << setiosflags(std::ios::left);
	cout << "List of parameters:" << endl;
	int n = 0;
	for (auto p : *this) {
		cout << p->tostring() << endl;
		if (p->_isUserDefined) n++;
	}

	if (!hasErrors())
	{
		if (n > 0) {
			cout << setw(10) << "Mode: ";
			if (isHelpRequired()) {
				cout << "Printing help" << endl;
			}
			else{
                if (isUserDefined(ParamCompObject::solve)) {
                    string SolverName = getParamValue(ParamCompObject::solver);
                    if (isUserDefined(ParamCompObject::initsolver) && getParamValue(ParamCompObject::initsolver)!="None")
						SolverName = getParamValue(ParamCompObject::initsolver) + "+" + getParamValue(ParamCompObject::solver);
					cout << "Instance " << getParamValue(ParamCompObject::instance) << " will be solved with " << SolverName << endl;
				}
			}
		}
	}
	else displayErrors();

	cout << resetiosflags(std::ios::adjustfield);
	cout << setiosflags(std::ios::right);
}


ParameterComponent* Parameter::getParamComponent(ParamCompObject P)
{
	ParameterComponent* PC = NULL;
	for (auto p : *this)  if (p->_pco == P) {
		PC = p;
		break;
	}
	assert(PC != NULL);
	return PC;
}

bool Parameter::checkFile(string filepath)
{
	ifstream input(filepath.c_str());
	if (!input.is_open()) return false;
	else {
		input.close();
		return true;
	}
}

string Parameter::getParamName(ParamCompObject P)
{
	ParameterComponent* PC = getParamComponent(P);
	return PC->_name;
}

inline void Parameter::validateIsLowerThan(ParamCompObject L, ParamCompObject R)
{
	if (getParamIntValue(L) > getParamIntValue(R)) {
		ostringstream Error;
		Error << "Parameter -" << getParamName(L) << " must be lower than or equals to parameter -" << getParamName(R);
		_ErrorMessage.push_back(Error.str());
	}
}

void Parameter::validateIsStrictlyLowerThan(ParamCompObject L, ParamCompObject R)
{
	if (getParamIntValue(L) >= getParamIntValue(R)) {
		ostringstream Error;
		Error << "Parameter -" << getParamName(L) << " must be strictly lower than parameter -" << getParamName(R);
		_ErrorMessage.push_back(Error.str());
	}
}

void Parameter::validateParametersList(int argc, char* argv[])
{
	for (int k = 1; k < argc; ++k) {
		string input(argv[k]);
		input.erase(0, 1);	//Removing the "-"
		size_t pos = input.find("=");
		string paramname = input.substr(0, pos);

		bool found = false;
		for (auto P : *this) {
			if (P->_name == paramname) found = true;
		}
		if (!found) {
			ostringstream Error;
			Error << "Parameter -" <<paramname << " is unknown";
			_ErrorMessage.push_back(Error.str());
		}
	}
}

void Parameter::validateRange(ParamCompObject L, int min, int max)
{	
	if (!getParamComponent(L)->isValInteger()) {
		ostringstream Error;
		Error << "Parameter -" << getParamName(L) << " must have an integer input";
		_ErrorMessage.push_back(Error.str());
	}
	else {
		if (getParamIntValue(L) < min || getParamIntValue(L) > max) {
			ostringstream Error;
			if (max >= INFINITY && getParamIntValue(L) <= max) {
				Error << "Parameter -" << getParamName(L) << " must be greater than or equal to " << min;
			}
			else
				Error << "Parameter -" << getParamName(L) << " must be between " << min << " and " << max;
			_ErrorMessage.push_back(Error.str());
		}
	}
}

void Parameter::validateFile(ParamCompObject L)
{
	string FolderPath = getParamValue(ParamCompObject::folderpath);
	if (L == ParamCompObject::datafile && isUserDefined(ParamCompObject::geninst)) {
		FolderPath = "";
	}
	if (isUserDefined(L) && !checkFile(FolderPath+getParamValue(L))) {
		ostringstream Error;
		Error << "File " << getParamValue(L) << " of Parameter -" << getParamName(L) << " not found in "<<FolderPath;
		_ErrorMessage.push_back(Error.str());
	}
}

Parameter::Parameter(int argc, char* argv[]) : vector<ParameterComponent*>()
{
	push_back(new ParameterComponent(ParamCompObject::help, "h", "", "Help"));
	push_back(new ParameterComponent(ParamCompObject::gendata, "d", "", "Data file generation mode"));
	push_back(new ParameterComponent(ParamCompObject::geninst, "i", "", "Instance file generation mode"));
	push_back(new ParameterComponent(ParamCompObject::solve, "s", "", "Solver mode"));
	push_back(new ParameterComponent(ParamCompObject::nhotels, "nhotels", "1", "Number of Hotels"));
	push_back(new ParameterComponent(ParamCompObject::ntours, "ntours", "10", "Number of Tours"));
	push_back(new ParameterComponent(ParamCompObject::ngroups,"ngroups", "10", "Number of Group to Serve"));
	push_back(new ParameterComponent(ParamCompObject::vcapacity, "vcapacity", "20", "Vehicles Capacity"));
	push_back(new ParameterComponent(ParamCompObject::mingsize, "mingsize", "1", "Minimum Size of a Group"));
	push_back(new ParameterComponent(ParamCompObject::maxgsize, "maxgsize", "10", "Maximum Size of a Group"));
	push_back(new ParameterComponent(ParamCompObject::minnodetour, "minnodetour", "1", "Minimum Number of Nodes in a Tour"));
	push_back(new ParameterComponent(ParamCompObject::maxnodetour, "maxnodetour", "5", "Maximum Number of Nodes in a Tour"));
	push_back(new ParameterComponent(ParamCompObject::mintrlength, "mintrlength", "180", "Minimum length of a Tour"));
	push_back(new ParameterComponent(ParamCompObject::maxtrlength, "maxtrlength", "600", "Maximum length of a Tour"));
	push_back(new ParameterComponent(ParamCompObject::mind, "mind", "15", "Minimum traversing time (minutes)"));
	push_back(new ParameterComponent(ParamCompObject::maxd, "maxd", "120", "Maximum traversing time (minutes)"));
	push_back(new ParameterComponent(ParamCompObject::planhor, "planhor", "1440", "Planning Horizon"));
	push_back(new ParameterComponent(ParamCompObject::seed, "seed", to_string(((int)clock()) % 1000), "Seed for Generator"));
	push_back(new ParameterComponent(ParamCompObject::QoS, "QoS", "90", "Quality of Service [0,100]"));
	push_back(new ParameterComponent(ParamCompObject::minl, "minl", "60", "Minimum Length of an Activity (minutes)"));
	push_back(new ParameterComponent(ParamCompObject::maxl, "maxl", "240", "Maximum Length of an Activity (minutes)"));
	push_back(new ParameterComponent(ParamCompObject::timlim, "timlim", "60", "Time Limit for the Solver"));
	push_back(new ParameterComponent(ParamCompObject::instance, "instance", "", "Instance File"));
	push_back(new ParameterComponent(ParamCompObject::datafile, "datafile", "", "Data File to generate an instance"));
	push_back(new ParameterComponent(ParamCompObject::folderpath, "folderpath", "Instances"+ string(PATHSEP), "Folder Path for Generated File"));
	push_back(new ParameterComponent(ParamCompObject::solver, "solver", "Greedy", "Solver mode (Greedy, MILP, CP, ...)"));
	push_back(new ParameterComponent(ParamCompObject::initsolver, "initsolver", "None", "Hybrid Mode: Solver called first"));
	push_back(new ParameterComponent(ParamCompObject::avrep, "avrep", "2", "Average node repitition in tours"));

	assert(argc > 0);
	if (argc == 1) {
		string file(argv[0]);
		size_t found = file.find_last_of("/\\");
		ostringstream Error;
		Error << "No parameter has been specified, if you need help, type: ./" << file.substr(found + 1) << " -h";
		_ErrorMessage.push_back(Error.str());
	}
	else {
		validateParametersList(argc, argv);
		for (auto P : *this) {
			int n=P->extractdata(argc, argv);
			if (n > 1) {
				ostringstream Error;
				Error << "Parameter -"<<P->_name<<" defined several times";
				_ErrorMessage.push_back(Error.str());
			}
		}
	}

	//Data Type Validation
	validateRange(ParamCompObject::QoS, 0, 100);
	validateRange(ParamCompObject::mind, 5);
	validateRange(ParamCompObject::minl, 30);
	validateRange(ParamCompObject::mingsize, 1);
	validateRange(ParamCompObject::seed, 0);
	validateRange(ParamCompObject::avrep, 1);

	validateIsLowerThan(ParamCompObject::mingsize, ParamCompObject::maxgsize);
	validateIsLowerThan(ParamCompObject::minnodetour, ParamCompObject::maxnodetour);
	validateIsLowerThan(ParamCompObject::minl, ParamCompObject::maxl);
	validateIsLowerThan(ParamCompObject::mind, ParamCompObject::maxd);
	validateIsLowerThan(ParamCompObject::maxgsize, ParamCompObject::maxgsize);
	validateIsLowerThan(ParamCompObject::maxgsize, ParamCompObject::vcapacity);
	validateIsLowerThan(ParamCompObject::maxd, ParamCompObject::planhor);
	validateIsLowerThan(ParamCompObject::maxl, ParamCompObject::maxtrlength);
	validateIsLowerThan(ParamCompObject::minl, ParamCompObject::maxl);

	//Parameters Compatibility Validation
	if (!isUserDefined(ParamCompObject::help) && !isUserDefined(ParamCompObject::gendata) && !isUserDefined(ParamCompObject::geninst) && !isUserDefined(ParamCompObject::solve)) {
		ostringstream Error;
		Error << "One operation mode must be specified: " << getParamComponent(ParamCompObject::help)->_name<<" / "
			<< getParamComponent(ParamCompObject::gendata)->_name << " / "
			<< getParamComponent(ParamCompObject::geninst)->_name << " / "
			<< getParamComponent(ParamCompObject::solve)->_name;
		_ErrorMessage.push_back(Error.str());
	}
	else if (isUserDefined(ParamCompObject::help)) {
		//Help must be asked without any other parameters
		for (auto P : *this) checkCompatibility(ParamCompObject::help, P->_pco);
	}
	else if (isUserDefined(ParamCompObject::solve)) {
		//Solve Instance Mode
		checkCompatibility(ParamCompObject::solve, ParamCompObject::help);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::gendata);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::geninst);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::minnodetour);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::maxnodetour);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::mind);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::maxd);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::mintrlength);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::maxtrlength);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::planhor);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::minl);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::maxl);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::nhotels);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::ntours);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::mingsize);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::maxgsize);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::datafile);
		checkCompatibility(ParamCompObject::solve, ParamCompObject::avrep);

		if (!isUserDefined(ParamCompObject::instance)) {
			ostringstream Error;
			Error << "Parameter -" << getParamComponent(ParamCompObject::solve)->_name << " requires parameter -" << getParamComponent(ParamCompObject::instance)->_name;
			_ErrorMessage.push_back(Error.str());
		}
		else {
			validateFile(ParamCompObject::instance);
		}
	}

	srand(getParamIntValue(ParamCompObject::seed));

}

Parameter::~Parameter()
{
	while (!this->empty()) {
		ParameterComponent* P = this->back();
		this->pop_back();
		delete P;
	}
}
