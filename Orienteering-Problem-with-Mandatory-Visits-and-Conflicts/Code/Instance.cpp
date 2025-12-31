#include "Instance.h"

const double Node::timefactor = 1;

Node::Node() {
    _id = -1;
    _x = 0;
    _y = 0;
    _st = 0;
    _s = -1;
}

string Node::tostring() {
    return "["+to_string(_id)+": ("+to_string(_x)+","+to_string(_y)+")  s = " + to_string(_s)+"]";
}

double Node::distanceto(Node *N){
    double d = (sqrt(pow(_x-N->_x,2)+pow(_y-N->_y,2)))*10;
	//int rd = round(d);
	//assert(fabs(rd - d) < 0.5);
    return /*ceil*/(d*timefactor);
}

void Instance::initcompatibilities(int size)
{
	int origin = 0;
	int destination = _nnodes - 1;
	if (size < 2) return;
	if (size == 2) {
		for (int i = 0; i < _nnodes; i++) if (i != origin && i != destination) {
			for (int j = i + 1; j < _nnodes; j++) if (j != origin && j != destination) {

				bool arecomp = false;
				for (int k = 0; k < _nvehicles; k++) {
					if (comp(k, i) && comp(k, j)) {
						arecomp = true;
						break;
					}
				}
				if (!arecomp) {
					_conflictnodesets.push_back(set<int>());
					_conflictnodesets.back().insert(i);
					_conflictnodesets.back().insert(j);
				}
				else {
					double dij = distance(_nodes[origin], _nodes[i]) + _nodes[i]->serviceTime()
						+ distance(_nodes[i], _nodes[j]) + _nodes[j]->serviceTime()
						+ distance(_nodes[j], _nodes[destination]);
					double dji = distance(_nodes[origin], _nodes[j]) + _nodes[j]->serviceTime()
						+ distance(_nodes[j], _nodes[i]) + _nodes[i]->serviceTime()
						+ distance(_nodes[i], _nodes[destination]);
					if (dij > _tmax && dji > _tmax) {
						_conflictnodesets.push_back(set<int>());
						_conflictnodesets.back().insert(i);
						_conflictnodesets.back().insert(j);
					}

				}
			}
		}
	}
	else {
		initcompatibilities(size - 1);
		vector< pair<int,int> > pairs;
		for (auto const& p : _conflictnodesets) if (p.size() == 2) {
			pairs.push_back(pair<int, int>(*p.begin(), *p.rbegin()));
		}

		vector< set<int> > newsets;
		for (int s = 0; s < _nnodes; ++s) if (s != origin && s != destination) {
			for (auto const& p : _conflictnodesets) if (p.size() == size - 1 && s < *p.begin()) {
				
				bool haspairwiseincompatibility = true;
				for (int i : p) {
					if (find(pairs.begin(), pairs.end(), make_pair(i, s)) == pairs.end()) {
						haspairwiseincompatibility = false;
						break;
					}
				}

				if (haspairwiseincompatibility) {
					set<int> p2 = p;
					p2.insert(s);
					//assert(find(newsets.begin(), newsets.end(), p2) == newsets.end());
					newsets.push_back(p2);
					assert(p2.size() == size);
				}
			}
			
		}

		_conflictnodesets.insert(_conflictnodesets.end(), newsets.begin(), newsets.end());

	}
		
}

void Instance::transformtosymmetricdistances()
{
    // Classic ATSP to TSP transformation (Jonker-Volgenant)
    // 1. For each non-depot node, create a duplicate (in-node and out-node)
    // 2. Update _nodes, _nnodes, _dist, and _comp accordingly
    // 3. Initial depot (0) and final depot (_nnodes-1) are not duplicated
    // 4. After transformation, final depot remains the last node

    int origin = 0;
    int destination = _nnodes - 1;
    int n_orig = _nnodes;

    // Step 1: Prepare new node list
    std::vector<Node*> new_nodes;
    new_nodes.reserve(origin + (n_orig - 2) * 2 + 1); // 0 + 2*(n-2) + 1

    // Add original depot
    new_nodes.push_back(_nodes[origin]);

    // For each non-depot node, add in-node and out-node
    for (int i = 1; i < destination; ++i) {
        // Out-node (copy of original)
        Node* out_node = new Node(*_nodes[i]);
        out_node->_id = static_cast<int>(new_nodes.size());
        new_nodes.push_back(out_node);

        // In-node (copy of original)
        Node* in_node = new Node(*_nodes[i]);
        in_node->_id = static_cast<int>(new_nodes.size());
		//in_node->_s = 0; // Reset score for in-nodes
        new_nodes.push_back(in_node);
    }

    // Add final depot
    new_nodes.push_back(_nodes[destination]);

    // Step 2: Build new distance matrix
    int n_new = static_cast<int>(new_nodes.size());
    double** new_dist = new double*[n_new];
	//double inf = std::numeric_limits<double>::infinity();
	double inf = this->tmax();
    for (int i = 0; i < n_new; ++i) {
        new_dist[i] = new double[n_new];
        std::fill(new_dist[i], new_dist[i] + n_new, inf);
    }

    // Fill distances
    // 1. Depot to out-nodes
    for (int i = 1; i < destination; ++i) {
        int out_idx = 2 * (i - 1) + 1;
        new_dist[origin][out_idx] = _dist[origin][i];
    }

    // 2. In-nodes to depot
    for (int i = 1; i < destination; ++i) {
        int in_idx = 2 * (i - 1) + 2;
        new_dist[in_idx][n_new - 1] = _dist[i][destination];
    }

    // 3. Out-node to in-node (0 cost, force visit order)
    for (int i = 1; i < destination; ++i) {
        int out_idx = 2 * (i - 1) + 1;
        int in_idx = 2 * (i - 1) + 2;
        new_dist[out_idx][in_idx] = 0; // Force visit order
    }

    // 4. In-node to out-node (for all pairs i != j)
    for (int i = 1; i < destination; ++i) {
        int in_idx_i = 2 * (i - 1) + 2;
        for (int j = 1; j < destination; ++j) {
            if (i == j) continue;
            int out_idx_j = 2 * (j - 1) + 1;
            new_dist[in_idx_i][out_idx_j] = _dist[i][j];
        }
    }

    // 5. Depot to final depot (if direct connection exists)
    new_dist[origin][n_new - 1] = _dist[origin][destination];

    // 6. Out-nodes to final depot
    for (int i = 1; i < destination; ++i) {
        int out_idx = 2 * (i - 1) + 1;
        new_dist[out_idx][n_new - 1] = _dist[i][destination];
    }

    // 7. Depot from in-nodes (if needed)
    for (int i = 1; i < destination; ++i) {
        int in_idx = 2 * (i - 1) + 2;
        new_dist[in_idx][n_new - 1] = _dist[i][destination];
    }

    // Step 3: Update _comp to include both out-node and in-node for each original node
    list<int>* new_comp = new list<int>[_nvehicles];
    for (int k = 0; k < _nvehicles; ++k) {
        for (int i : _comp[k]) {
            if (i == 0 || i == n_orig - 1) {
                // Depot nodes remain at index 0 and n_new-1
                if (i == 0)
                    new_comp[k].push_back(0);
                else
                    new_comp[k].push_back(n_new - 1);
            } else {
                // Out-node index
                int out_idx = 2 * (i - 1) + 1;
                // In-node index
                int in_idx = 2 * (i - 1) + 2;
                new_comp[k].push_back(out_idx);
                new_comp[k].push_back(in_idx);
            }
        }
    }

    // Step 4: Clean up old nodes (except depots)
    for (int i = 1; i < destination; ++i) {
        delete _nodes[i];
    }
    delete[] _nodes;

    // Step 5: Clean up old distance matrix
    for (int i = 0; i < n_orig; ++i) {
        delete[] _dist[i];
    }
    delete[] _dist;

    // Step 6: Clean up old _comp
    delete[] _comp;
    _comp = new_comp;

    // Step 7: Assign new structures
    _nodes = new Node*[n_new];
    for (int i = 0; i < n_new; ++i) {
        _nodes[i] = new_nodes[i];
    }
    _nnodes = n_new;
    _dist = new_dist;

	// Step 8: update the whole dist matrix to assure d[i[[j]] = d[j][i] for all i,j
	for (int i = 0; i < n_new; ++i) {
		for (int j = 0; j < n_new; ++j) {
			if (new_dist[i][j] != inf && new_dist[j][i] == inf) {
				new_dist[j][i] = new_dist[i][j];
			}
		}
	}

}

Instance::Instance(string filename, string type) {
	_type = type;
	_name = filename;
	_nnodes = 0;
	_nvehicles = 0;
	ifstream input(filename.c_str(), ios::in);
	if(!input.is_open()){
		cout << "Error: File " << filename << " not found" << endl;
		exit(1000);
	}

	string buffer;
	input >> buffer >> _nnodes;
	assert(buffer == "n");
	//cout << buffer << _nnodes << endl;

	input >> buffer >> _nvehicles;
	assert(buffer == "m");
	//cout << buffer << _nvehicles << endl;

	input >> buffer >> _tmax;
	assert(buffer == "tmax");
	//cout << buffer << _tmax << endl;

	_nodes = new Node*[_nnodes];
	_comp = new list<int>[_nvehicles];

	for(int i = 0; i < _nnodes; ++i) {
		_nodes[i]=new Node();
		_nodes[i]->_id = i;
		if (_type == "implicit" ){
			input >> _nodes[i] -> _x >> _nodes[i]->_y >> _nodes[i]-> _s;
		}
		else input >> _nodes[i] -> _st >> _nodes[i]->_s;
		for (int k = 0; k < _nvehicles; k++) {
			input >> buffer;
			//cout << buffer << endl;
			if ( stoi(buffer) == 1 && i != 0 && i != _nnodes - 1)
				_comp[k].push_back(i);
		}
	}

	if ( _type == "explicit" ) {
		_dist = new double*[_nnodes];
		for(int i = 0; i < _nnodes; ++i) {
			_dist[i]= new double[_nnodes];
			for(int j=0; j < _nnodes; j++) {
				input >> _dist[i][j];
				//cout << _dist[i][j] << endl;
				_dist[i][j] = round(_dist[i][j]*10); //re-scale
			}
		}
		transformtosymmetricdistances();
	}
	_tmax = _tmax*10; // re-scale





	_fastcomp= new unordered_set<int>[_nvehicles];
	for (int k = 0; k < _nvehicles; k++)
		_fastcomp[k].insert(_comp[k].begin(), _comp[k].end());


	//Init the set of reachable nodes for _tmax
	int origin = 0;
	int destination = _nnodes - 1;
	for (int i = 0; i < _nnodes ; i++) {
		
		double d = distance(_nodes[origin], _nodes[i]) + _nodes[i]->serviceTime() + distance(_nodes[i], _nodes[destination]);
		if (d <= _tmax) {
			_reachablenodes.push_back(i);
		}
	}

	//Init the set of conflict node pairs
	int size = _nvehicles;
	if (_type == "explicit") size = 2; // for explicit instances, we only need pairs of nodes
	initcompatibilities(size);
	cout << "Size of conflict node pairs: " << _conflictnodesets.size() << endl;
}

Instance::~Instance()
{
    for(int i = 0; i < _nnodes; ++i) delete _nodes[i];
    delete [] _nodes;

    delete [] _comp;

    if (_type == "explicit"){
    	for(int i = 0; i < _nnodes; ++i) delete _dist[i];
    	delete [] _dist;
    }
}

double Instance::distance(Node* O, Node* D){
	if (_type == "implicit") return O->distanceto(D);
	else return _dist[O->index()][D->index()];
}

bool Instance::comp(int v, int i) {
	if (i == 0 || i >= nnodes() - 1) return true;
	return _fastcomp[v].find(i) != _fastcomp[v].end();
	//return find(_fastcomp[v].begin(), _fastcomp[v].end(), i) != _fastcomp[v].end();
	//return find(_comp[v].begin(), _comp[v].end(), i)!= _comp[v].end();
}

bool Instance::fastcomp(int v, int i) {
	if (i == 0 || i >= nnodes() - 1) return true;
	return _fastcomp[v].find(i) != _fastcomp[v].end();
	
}

double Instance::totalScore(){
	double totalScore = 0;
	for (int i=1; i < _nnodes -1; i++) {
		totalScore += _nodes[i]->score();
	}
    return totalScore;
}

void Instance::print()
{
    cout << "Instance: " << _name << endl;
    cout << "# vehicles: " << _nvehicles << endl;
    cout << "# nodes: " << _nnodes << endl;
    cout << "Tmax: " << _tmax << endl;
    for(int i = 0; i < _nnodes; i++){
        cout << _nodes[i]->tostring() << endl;
    }

    for (int k = 0; k < _nvehicles; k++) {
        for (auto const &i: _comp[k]) {
            std::cout << i << "\t";
        }
        std::cout << std::endl;
    }

    // only the first 10 rows
    for (int i = 0; i < 10; i++) {
    	for (int j = 0; j < _nnodes; j++) {
    		if (_type == "implicit") cout << distance(node(i), node(j)) << setw(8);
    		else cout << _dist[i][j] << setw(8);
    	}
    	cout << endl;
    }
}

void Instance::print_costs(string file) {
	string fileName = "distances_" + file;
	ofstream a_file(fileName.c_str());
	for (int i = 0; i < _nnodes; i++) {
		for (int j = 0; j < _nnodes; j++) {
			//a_file << setw(8) << _dist[i][j];
			a_file << setw(8) << distance(node(i), node(j));;
		}
		a_file << endl;
	}
}

double Instance::score_comp(int r) { // average possible score
	double tsc = 0; // total score compatible
	list<int> compatibles = compRoute(r);
	for (list<int>::iterator it = compatibles.begin(); it!= compatibles.end(); it++) {
		tsc += _nodes[*it]->score();
	}
	return tsc;
}

