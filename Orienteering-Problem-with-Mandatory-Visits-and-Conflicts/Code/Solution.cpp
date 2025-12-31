#include "Solution.h"

Solution::Solution(Instance* I)
{
	_I = I;
	_route = new list<int>[_I->nvehicles()];
}

Solution::~Solution()
{
	delete[] _route;
}

void Solution::addNode(int k, int i) {
	_route[k].push_back(i);
}

bool Solution::isFeasible(bool saveinfile)
{
	vector<string> ErrorMessages;

   // One visit per node
	int visits = 0;
	for (int i = 1; i < _I->nnodes()-2; i++) {
		visits = 0;
		list<int>::iterator it;
		for (int k = 0; k < _I->nvehicles(); k++) {
			it = find(_route[k].begin(), _route[k].end(), i);
			if (it != _route[k].end())
				visits += 1;
		}
		if (visits > 1) {
			ErrorMessages.push_back("Infeasible solution: node " +to_string(i) + " visited more than once");
		}
	}

	// Start and end of the routes
    int start = 0;
    int end = 0;
    for (int k = 0; k < _I->nvehicles(); k++) if(!_route[k].empty()) {
    	if (_route[k].front() == 0)
    		start += 1;
    	else {
			ErrorMessages.push_back("Infeasible solution: route " + to_string(k) + " do not start at deposit 0");
    	}
    	if (_route[k].back() == (_I->nnodes() - 1))
    		end += 1;
    	else {
			ErrorMessages.push_back("Infeasible solution: route " + to_string(k) + " do not end at deposit ");
    	}
    }

    // Time limit
	double time = 0;
	double st = 0;
	for (int k = 0; k < _I->nvehicles(); k++) {
		time = 0;
		st = 0;
    	for(list<int>::iterator it = _route[k].begin(); it != _route[k].end(); it++) {
    		Node* O = _I->node(*it);
    		st += O->serviceTime();
    		if (*it != _route[k].back()) {
    			list<int>::iterator it2 = it;
    			advance(it2, 1);
    			Node* D = _I->node(*it2);
    			time += _I->distance(O, D);
    		}
    	}
        if (time + st > _I->tmax()) {
			ErrorMessages.push_back("Infeasible solution: route " + to_string(k) + " has a duration of " + to_string(time + st));
		}
	}

	// Compatibilities
    for (int k = 0; k < _I->nvehicles(); k++) {
        for(list<int>::iterator it = _route[k].begin(); it != _route[k].end(); it++) {
        	if (*it != 0 && *it != (_I->nnodes() - 1)) {
        		if( !_I ->comp(k, *it)) {
					ErrorMessages.push_back("Infeasible solution: customer " + to_string(*it) + " is not compatible with route " + to_string(k));
        		}
        	}
        }
    }

	// Subtours
	for (int k = 0; k < _I->nvehicles(); k++) {
		for (list<int>::iterator it = _route[k].begin(); it != _route[k].end(); it++) {
			if (it != _route[k].begin()) {
				list<int>::iterator it2 = it;
				list<int>::iterator it3;
				advance(it2, 1);
				it3 = find(it2, _route[k].end(), *it);
				if (it3 != _route[k].end()) {
					ErrorMessages.push_back("Infeasible solution: subtour in route " + to_string(k));
					break;
				}
			}
		}
	}

	if (saveinfile) {
		// save errors in a file
		fstream file;
		file.open("Errors.txt", ios::in);
		if (file.is_open()) {
			for (auto& E : ErrorMessages)
				file << E << endl;
			file.close();
		}
	}
		

	for (auto& E : ErrorMessages) cout << E << endl;
	
	return ErrorMessages.empty();
}

void Solution::saveSolution(double objvalue, int OF) {
	string a = "Sol_" + _I->name();
	//ofstream file(a.c_str(), ios::app);

	int w = 7;
	string obj;
	if (OF == 1) obj = "Max-Min";
	else obj = "Min-Max";
	string d = "Objective value (" + obj + "): " + to_string(objvalue);

	fstream file;
	file.open(a.c_str(), ios::in);
	if (file.is_open()) {
		ofstream file(a.c_str(), ios::app);
		file << d.c_str() << endl;
		for (int k = 0; k < _I->nvehicles(); k++) {
			string b = "Route " + to_string(k) + ":  ";
			file << b.c_str();
			for (list<int>::iterator it = _route[k].begin(); it != _route[k].end(); it++) {
				file << to_string(*it) << setw(w);
			}
			file << endl;
			file << "Score: " << get_ScoreAcum(k) << endl;
			file << "Time: " << get_Tacum(k) << endl;
			file << "No. Clients: " << getRoute(k).size() << endl;
		}
	} else {
		ofstream file(a.c_str());
		file << "Instance: " << _I->name() << endl;
		file << d.c_str() << endl;
		for (int k = 0; k < _I->nvehicles(); k++) {
			string b = "Route " + to_string(k) + ":  ";
			file << b.c_str();
			for (list<int>::iterator it = _route[k].begin(); it != _route[k].end(); it++) {
				file << to_string(*it) << setw(w);
			}
			file << endl;
			file << "Score: " << get_ScoreAcum(k) << endl;
			file << "Time: " << get_Tacum(k) << endl;
			file << "No. Clients: " << getRoute(k).size() << endl;
		}
	}

	/*file << d.c_str() << endl;
	for (int k = 0; k < _I->nvehicles(); k++) {
		string b = "Route " + to_string(k) + ":  ";
		file << b.c_str();
		for (list<int>::iterator it = _route[k].begin(); it != _route[k].end(); it++) {
			file << to_string(*it) << setw(w);
		}
		file << endl;
	}*/
	file.close();
}

void Solution::printSolution() {
	cout << "Min Score: " << minScore() << endl;
	int w = 7;
	for (int k = 0; k < _I->nvehicles(); k++) {
		cout << "Route " << to_string(k) << ": ";
        //int ct=0;
		for (list<int>::iterator it = _route[k].begin(); it != _route[k].end(); it++) {
			cout << setw(w) << to_string(*it); //+ " algo esta mal<"+to_string(ct) + "> ";
            /*list<int>::iterator itnext=it;
            itnext++;
            if(itnext!=_route[k].end()){
                ct+=_I->distance(_I->node(*it), _I->node(*itnext));
            }*/
		}
		cout << endl;
	}
}

double Solution::totalScoreK(int k) {
	double totalS = 0;
	for(list<int>::iterator it = _route[k].begin(); it != _route[k].end(); it++) {
		totalS += _I -> node(*it)->score();
	}
	return totalS;
}

double Solution::minScore() {
	int m = _I -> nvehicles();
	double* scores=new double[m];
	for (int k = 0; k < m; k++) {
	    scores[k] = totalScoreK(k);
	}

	double min_score = scores[0];
	for (int i = 0; i < m; i++){
		if (scores[i] < min_score){
			min_score = scores[i];
		}
	}

	delete[] scores;
	return min_score;
}

double Solution::maxDev() {
	int m = _I -> nvehicles();
	double* devs=new double[m];
	double avgScore = 1.0 * (_I -> totalScore())/m;
    for (int k = 0; k < m; k++) {
    	devs[k] = fabs(avgScore - totalScoreK(k));
    }

    //devs.sort();
    double max_dev = devs[0];
    for (int i = 0; i < m; i++){
        if (devs[i] > max_dev){
        	max_dev = devs[i];
        }
    }
	delete[] devs;

	return max_dev;
}

int Solution::get_ScoreAcum(int k) {
	int sacum = 0;
   	for(list<int>::iterator it = _route[k].begin(); it != _route[k].end(); it++) {
   		Node* D = _I->node(*it);
   		sacum += (int)D -> score();
	}
   return sacum;
}

double Solution::get_Tacum(int k) {
	double tacum = 0.0;
   	for(list<int>::iterator it = _route[k].begin(); it != _route[k].end(); it++) {
    	list<int>::iterator it2 = it;
    	advance(it2, 1);
    	if (it2 != _route[k].end())
			tacum += _I->distance(_I->node(*it), _I->node(*it2));
	}
   return tacum;
}

list<int> Solution::getRoute(int k) {
	return _route[k];
}

void Solution::addNode_pos(int k, int id, int p) {
	list<int>::iterator it = _route[k].begin();
	list<int>::iterator it2 = it;
	advance(it2, p);
	_route[k].insert(it2, id);
}
