#include "Instance.h"

double TwoDimNode::_corr = 1.0;


string to_string(NodeType T)
{
	if (T == NodeType::pickup) return "pickup";
	else if (T == NodeType::delivery) return "delivery";
	else return "n/a";
}

int spot::_idgen = 0;
int tour::_idgen = 0;
int group::_idgen = 0;
int node::_idgen = 0;


spot::spot(int spotid, int lengthofstay, int a, int b, string spotname) {
	_id = _idgen++;
	_spotid = spotid;
	_lengthofstay = lengthofstay;
	if (spotname == "") _spotname = "N_" + to_string(_spotid);
	else _spotname = spotname;
	_a = a;
	_b = b;
}


tour::tour(int trid, string tourname) : vector<spot*>() {
	_id = _idgen++;
	_trid = trid;
	if (_trname == "") _trname = "tour_" + to_string(_trid);
	else _trname = tourname;
}


group::group(int grid, int grsize, tour* tour, spot* origin) {
	_id = _idgen++;
	_grid = grid;
	_grsize = grsize;
	_tour = tour;
	_origin = origin;
	assert(_tour != NULL);
	assert(_origin != NULL);
	_maxtrlength = INT_MAX;
	_mintrlength = INT_MAX;
	_firstgroupnode = NULL;
}


string node::extendedname()
{
	string s="N_"+to_string(_id)+" [";
	if (_type == NodeType::pickup) s += "P";
	else s += "D";
	s += " gr=" + to_string(_group->_grid) + "(" + to_string(_group->_id) + ")";
	s += " sp=" + to_string(_spot->_spotid) + "(" + to_string(_spot->_id) + ")";
	s += " tr=" + to_string(_group->_tour->_trid) + "(" + to_string(_group->_tour->_id) + ")";
	s += " a=" + to_string(_spot->_a);
	if (_spot->_b < INT_MAX) s +=" b="+to_string(_spot->_b);
	else s += " b=inf";
	s += "]";
	return s;
}

node::node(group* group, spot* spot, NodeType type)
{
	_id = _idgen++;
	_group = group;
	_spot = spot;
	_prev = _next = NULL;
	_type = type;
}


void Instance::resetallidgen()
{
	node::_idgen = 0;
	tour::_idgen = 0;
	group::_idgen = 0;
	spot::_idgen = 0;
}

void Instance::skipFileComments(ifstream* input)
{
	input->ignore(numeric_limits<streamsize>::max(), '#');
	int c = input->peek();
	assert(!input->eof());
	do{
		input->ignore(numeric_limits<streamsize>::max(), '\n');
	} while (input->peek() == '#');
}

void Instance::initnodes()
{
	_nnodes = 0;
	for (int g = 0; g < _ngroups; ++g) _nnodes += 2*_groups[g]->_tour->size() + 2;	//The +2 is for origin and destination
	_nodes = new node * [_nnodes];
	int k = 0;
	for (int g = 0; g < _ngroups; ++g) {
		tour* t= _groups[g]->_tour;
		spot* origin = _groups[g]->_origin;
		node* PrevNode = new node(_groups[g], origin, NodeType::pickup);
		_groups[g]->_firstgroupnode = PrevNode;
		_nodes[k++] = PrevNode;
		for (tour::iterator it = t->begin(); it != t->end(); it++) {
			spot* spot = *it;

			//delivery
			node* Nd = new node(_groups[g], *it, NodeType::delivery);
			PrevNode->_next = Nd;
			Nd->_prev = PrevNode;
			_nodes[k++] = Nd;
			PrevNode = Nd;

			//pickup
			node* Np = new node(_groups[g], *it, NodeType::pickup);
			PrevNode->_next = Np;
			Np->_prev = PrevNode;
			_nodes[k++] = Np;
			PrevNode = Np;
		}
		node* LastNode = new node(_groups[g], origin, NodeType::delivery);
		PrevNode->_next = LastNode;
		LastNode->_prev = PrevNode;
		_nodes[k++] = LastNode;
	}
	assert(k == _nnodes);
}

void Instance::read()
{
	string filepath = _Param->getParamValue(ParamCompObject::folderpath) + _Param->getParamValue(ParamCompObject::instance);
	ifstream input(filepath.c_str());
	assert(input.is_open());

	skipFileComments(&input);

	_vcapacity = _Param->getParamIntValue(ParamCompObject::vcapacity);
	_planhor= _Param->getParamIntValue(ParamCompObject::planhor);

	input >> _ngroups >> _ntours >> _nspots;


	skipFileComments(&input);

	//The spots
	bool isidsremap = false;
	_spots = new spot * [_nspots];
	int posinfonodes = input.tellg();
	map<int, int> idsmapping;
	for (int i = 0; i < _nspots; ++i) {
		_spots[i] = new spot(0, 0);
		string buffer;
		input >> _spots[i]->_spotid >> _spots[i]->_spotname >> _spots[i]->_lengthofstay >> _spots[i]->_a >> _spots[i]->_b;
		if (i != _spots[i]->_spotid) isidsremap = true;
	}

	skipFileComments(&input);

	//The tours
	_tours = new tour * [_ntours];
	for (int t = 0; t < _ntours; ++t) {
		_tours[t] = new tour(0);
		int size = 0;
		input >> _tours[t]->_trid >> _tours[t]->_trname >> size;
		for (int k = 0; k < size; ++k) {
			int spotid = 0;
			input >> spotid;
			int i = 0;
			while (i < _nspots && _spots[i]->_spotid != spotid) i++;
			assert(i < _nspots);
			_tours[t]->push_back(_spots[i]);
		}
		reverse(_tours[t]->begin(), _tours[t]->end());
	}

	skipFileComments(&input);

	//The groups
	_groups = new group * [_ngroups];
	for (int g = 0; g < _ngroups; ++g) {
		int grid;
		int gsize;
		int trid;
		int origin;
		
		input >> grid >> gsize >> trid >> origin;
		
		int t = 0;
		while (t < _ntours && _tours[t]->_trid != trid) t++;
		assert(t < _ntours);

		int i = 0;
		while (i < _nspots && _spots[i]->_spotid != origin) i++;
		assert(i < _nspots);

		_groups[g] = new group(grid, gsize, _tours[t], _spots[i]);
	}

	skipFileComments(&input);

	//The time matrix
	_timematrix = new int*[_nspots];
	for (int i = 1; i < _nspots; ++i) {
		_timematrix[i] = new int[i];
		string buffer;
		input >> buffer;
		for (int j = 0; j < i; ++j) {
			input >> _timematrix[i][j];
		}
	} 

	//The nodes
	initnodes();


	if (isidsremap) {
		cout << "Warning: Node ids remapped to reduce memory usage:" << endl;
		cout << setw(15) << "Name" << setw(15) << "Original_ID" << setw(15) << "New_ID"  << endl;
		input.seekg(posinfonodes);
		for (int i = 1; i < _nspots; ++i) {
			int id = 0;
			string buffer;
			string name;
			input >> id >> name >> buffer >> buffer >> buffer;
			if(id!=i)
				cout << setw(15) << name << setw(15) << id << setw(15) << i << endl;
		}

	}

	input.close();
}

void Instance::save(string filename)
{
	int w = 15;
	int sw = 5;
	if (filename == "") filename = _Param->getParamValue(ParamCompObject::instance);
	string filepath = _Param->getParamValue(ParamCompObject::folderpath) + filename;
	ofstream output(filepath.c_str());
	assert(output.is_open());


	output << setw(sw) << "#" << setw(w) << "ngroups" << setw(w) << "ntours" << setw(w) << "nspots" << endl;
	output << setw(sw) << " " << setw(w) << _ngroups << setw(w) << _ntours << setw(w) << _nspots << endl;
	output << endl;

	//The Spots
	output << setw(sw) << "#" << setw(w) << "spot_id" << setw(3*w) << "n_name" << setw(w) << "length" << setw(w) << "a" << setw(w) << "b" << endl;
	for (int i = 0; i < _nspots; ++i) {
		spot* s = _spots[i];
		output << setw(sw) << " " << setw(w) << s->_spotid << setw(3*w) << s->_spotname << setw(w) << s->_lengthofstay << setw(w) << s->_a << setw(w) << s->_b << endl;
	}
	output << endl;

	//The tours
	output << setw(sw) << "#" << setw(w) << "tr_id" << setw(3*w) << "tr_name" << setw(w) << "nspot" << setw(w) << "seq" << endl;
	for (int t = 0; t < _ntours; ++t) {
		tour* T = _tours[t];
		output << setw(sw) << " " << setw(w) << T->_trid << setw(3*w) << T->_trname << setw(w) << T->size();
		for (auto s : *T) output << setw(w) << s->_spotid;
		output << endl;
	}
	output << endl;

	//The groups
	output << setw(sw) << "#" << setw(w) << "gr_id" << setw(w) << "gr_size" << setw(w) << "tr_id" << setw(10) << "origin" << endl;
	for (int g = 0; g < _ngroups; ++g) {
		group* G = _groups[g];
		output << setw(sw) << " " << setw(w) << G->_grid << setw(w) << G->_grsize << setw(w) << G->_tour->_trid << setw(10) << G->_origin->_spotid << endl;
	}
	output << endl;

	//The time matrix
	output << setw(sw) << "#" << setw(w) << "o/d";
	for (int i = 0; i < _nspots; ++i) output << setw(w) << _spots[i]->_spotid; output << endl;
	for (int i = 1; i < _nspots; ++i) {
		output << setw(sw) << " " << setw(w) << _spots[i]->_spotid;
		for (int j = 0; j < i; ++j)
			output << setw(w) << _timematrix[i][j];
		output << endl;
	}

	output.close();
}

void Instance::correctTimeMatrix()
{
	//Correcting triangular inequalities
	set<int> seti;
	for (int i = 1; i < _nspots; ++i) seti.insert(i);
	while (!seti.empty()) {
		int i = *seti.begin();
		seti.erase(i);
		assert(_spots[i]->_id == i);
		for (int j = 0; j < i; ++j) {
			int l = travellingtime(_spots[i], _spots[j]);//_timematrix[i][j];
			for (int k = 0; k < _nspots; ++k) if (k != i && k != j) {
				int l1 = travellingtime(_spots[i], _spots[k]);
				int l2 = travellingtime(_spots[k], _spots[j]);

				if (l1 + l2 < l) {
					_timematrix[i][j] = l1 + l2;
					seti.insert(i);
					seti.insert(j);
				}
			}
		}
	}

	//Checking the triangular inequalities
	for (int i = 1; i < _nspots; ++i) {
		for (int j = 0; j < i; ++j) {
			int l = travellingtime(_spots[i], _spots[j]);//_timematrix[i][j];
			for (int k = 0; k < _nspots; ++k) if (k != i && k != j) {
				int l1 = travellingtime(_spots[i], _spots[k]);
				int l2 = travellingtime(_spots[k], _spots[j]);

				assert(l1 + l2 >= l);
			}
		}
	}
}

string Instance::tostring(node *N)
{
	ostringstream s;
	s << N->_id << "(" << N->extendedname() << "): "; 
	if (N->_prev != NULL) s << "- o=" << N->_prev->_id;
	else s << "- o=X";
	s << " ";
	if (N->_next != NULL) s << "- d=" << N->_next->_id;
	else s << "- d=X";

	return s.str();
}

string Instance::tostring(group* G) {
	string s="G_"+to_string(G->_grid)+"("+to_string(G->_id)+"): ";
	s += "size=" + to_string(G->_grsize) + " org=" + G->_origin->_spotname + "(" + to_string(G->_origin->_id) + ")";
	s += " tr = " + G->_tour->_trname;
	s += " minl=" + to_string(G->_mintrlength);
	s += " maxl=" + to_string(G->_maxtrlength);

	return s;
}

string Instance::tostring(tour* T)
{
	string s = T->_trname + "(" + to_string(T->_id) + "): ";
	s += "size=" + to_string(T->size());
	s += " < ";
	for (auto sp : *T) {
		s += sp->_spotname + "(" + to_string(sp->_id) + ") ";
	}
	s += ">";
	return s;
}

void Instance::genDataFile(Parameter* Param)
{
	string DataFile = Param->getParamValue(ParamCompObject::datafile);
	string Folder = Param->getParamValue(ParamCompObject::folderpath);
	cout << "Generating Data File " << DataFile << " ..." << endl;

	double corr = Param->getParamIntValue(ParamCompObject::maxd);
	TwoDimNode::setcorr(corr);
	double prob = (double)Param->getParamIntValue(ParamCompObject::avrep) / (double)Param->getParamIntValue(ParamCompObject::ntours);

	vector<TwoDimNode*> Points;	//Points that belongs to at least one tour
	vector<TwoDimNode*> Pool;	//Pool of Points that DO NOT belong to any tours
	vector<TwoDimNode*> Hotels;
	vector< vector<TwoDimNode*> > Tours;

	//Creating the Pool
	double n = ceil((double)Param->getParamIntValue(ParamCompObject::maxd) / (double)Param->getParamIntValue(ParamCompObject::mind));
	double step = ((double)Param->getParamIntValue(ParamCompObject::mind) / (double)Param->getParamIntValue(ParamCompObject::maxd)) + 0.001;
	for (int i = 0; i <= n; ++i) for (int j = 0; j <= n; ++j) {
		double l = RAND(Param->getParamIntValue(ParamCompObject::minl), Param->getParamIntValue(ParamCompObject::maxl));
		l -= (int)l % 30;
		double x = i * step;
		double y = j * step;
		TwoDimNode* P = new TwoDimNode(x, y, l);
		Pool.push_back(P);
	}
	std::random_shuffle(Pool.begin(), Pool.end());

	cout << "Pool of " << Pool.size() << " nodes" << endl;

	//We construct the tours
	while (Tours.size() < Param->getParamIntValue(ParamCompObject::ntours)) {
		vector<TwoDimNode*> tour;
		int size = RAND(Param->getParamIntValue(ParamCompObject::minnodetour), Param->getParamIntValue(ParamCompObject::maxnodetour) + 1);
		assert(size > 0 && size <= Param->getParamIntValue(ParamCompObject::maxnodetour));

		//First we try to add existing points
		std::random_shuffle(Points.begin(), Points.end());
		for (auto P : Points) if (tour.size() < size && RAND01() <= prob) {
			tour.push_back(P);
		}

		//Then we add points from the pool
		for (auto P : Pool) if (tour.size() < size) {
			tour.push_back(P);
		}

		//Finally we validate the tour
		std::random_shuffle(tour.begin(), tour.end());

		while (TwoDimNode::length(tour) > Param->getParamIntValue(ParamCompObject::maxtrlength)) {
			tour.pop_back();
		}

		if (tour.size() < Param->getParamIntValue(ParamCompObject::minnodetour)
			|| TwoDimNode::length(tour) < Param->getParamIntValue(ParamCompObject::mintrlength))
			tour.clear();

		for (auto T : Tours) if (!tour.empty() && T == tour) {
			tour.clear();
			break;
		}

		if (!tour.empty()) {
			//We keep the tour
			Tours.push_back(tour);

			TwoDimNode::print(tour);

			//We move Nodes from Pool if required
			vector<TwoDimNode*> oldPool = Pool;
			Pool.clear();
			for (auto P : oldPool) {
				if (P->isPresent(tour)) Points.push_back(P);
				else Pool.push_back(P);
			}
		}
	}

	//We get the hotel from the rest
	for (auto P : Pool) if (Hotels.size() < Param->getParamIntValue(ParamCompObject::nhotels)) {
		bool feas = true;
		for (auto T : Tours) {
			vector< TwoDimNode*> gTour;
			gTour.push_back(P);
			for (auto S : T) gTour.push_back(S);
			gTour.push_back(P);

			if (TwoDimNode::length(gTour) > Param->getParamIntValue(ParamCompObject::planhor)) {
				feas = false;
				break;
			}
		}

		if (feas) {
			Hotels.push_back(P);
		}
	}

	//We complete the hotels with already used nodes
	std::random_shuffle(Points.begin(), Points.end());
	for (auto P : Points) if (Hotels.size() < Param->getParamIntValue(ParamCompObject::nhotels)) {
		bool feas = true;
		for (auto T : Tours) {
			vector< TwoDimNode*> gTour;
			gTour.push_back(P);
			for (auto S : T) gTour.push_back(S);
			gTour.push_back(P);

			if (TwoDimNode::length(gTour) > Param->getParamIntValue(ParamCompObject::planhor)) {
				feas = false;
				break;
			}
		}

		if (feas) Hotels.push_back(P);
	}

	cout << "# Hotels: " << Hotels.size() << " (Target= " << Param->getParamIntValue(ParamCompObject::nhotels) << ")" << endl;

	//We save everything in a file
	int nspot = Points.size();
	for (auto H : Hotels) if (!H->isPresent(Points)) Points.push_back(H);
	int w = 15;
	int sw = 5;
	string filepath = Folder + DataFile;
	ofstream output(filepath.c_str());
	assert(output.is_open());

	output << setw(sw) << "#" << setw(w) << "ntours" << setw(w) << "nnodes" << setw(w) << "nspots" << setw(w) << "nhotels" << endl;
	output << setw(sw) << " " << setw(w) << Tours.size() << setw(w) << Points.size() << setw(w) << nspot << setw(w) << Hotels.size() << endl;
	output << endl;

	output << setw(sw) << "#" << setw(w) << "NodeId" << setw(w) << "StayLength" << setw(w) << "NodeName" << setw(w) << "NodeType" << endl;
	int id = 0;

	for (auto P : Points) {
		string Name = "Spot_" + to_string(id);
		string Type = "Spot";
		int l = (int)P->l();
		if (P->isPresent(Hotels)) {
			Name = "Hotel_" + to_string(id);
			Type = "Hotel";
			bool isSpot = false;
			for (auto T : Tours) if (P->isPresent(T)) isSpot = true;
			if (isSpot) {
				Type = "Hotel/Spot";
			}
			else l = 0;
		}
		output << setw(sw) << " " << setw(w) << id++ << setw(w) << l << setw(w) << Name << setw(w) << Type << endl;
	}

	assert(id == Points.size());

	output << setw(sw) << "#" << setw(w) << "TourId" << setw(w) << "TourName" << setw(w) << "nspot" << setw(w) << "list_of_spots" << endl;
	int trid = 0;
	for (auto T : Tours) {
		output << setw(sw) << " " << setw(w) << trid++ << setw(w) << "Tour_" + to_string(trid - 1) << setw(w) << T.size();
		for (auto P : T) {
			int id = 0;
			while (id < Points.size() && Points[id] != P) id++;
			assert(id < Points.size());
			output << setw(w) << id;
		}
		output << endl;

	}

	output << setw(sw) << "#" << setw(w) << "Distance Matrix" << endl;
	output << setw(sw) << "#" << setw(w) << "o/d";
	for (int i = 0; i < Points.size(); ++i) output << setw(w) << i;
	output << endl;
	for (int i = 1; i < Points.size(); ++i) {
		output << setw(sw) << " " << setw(w) << i;
		for (int j = 0; j < i; j++) {
			int dist = (int)Points[i]->distanceto(*Points[j]);
			assert(dist >= Param->getParamIntValue(ParamCompObject::mind));
			output << setw(sw) << " " << setw(w) << Points[i]->distanceto(*Points[j]);
		}
		output << endl;
	}

	cout << "... " << DataFile << " created in Folder " << Folder << endl;

}

void Instance::genInstanceFile(Parameter* Param)
{
	//We open the datafile
	string datafile = Param->getParamValue(ParamCompObject::datafile);
	ifstream data(datafile.c_str());
	assert(data.is_open());

	skipFileComments(&data);
	int ntours, nnodes, nspots, nhptels;
	data >> ntours >> nnodes >> nspots >> nhptels;

	skipFileComments(&data);
	int posnodeinfo = data.tellg();
	int* NodeId = new int[nnodes];
	int* StayLength = new int[nnodes];
	string* NodeName = new string[nnodes];
	vector<int> Spots, Hotels;
	for (int i = 0; i < nnodes; ++i) {
		string type;
		data >> NodeId[i] >> StayLength[i] >> NodeName[i] >> type;
		if (type == "Spot") Spots.push_back(i);
		else if (type == "Hotel") Hotels.push_back(i);
		else {
			Spots.push_back(i);
			Hotels.push_back(i);
		}
	}

	skipFileComments(&data);
	int postourinfo = data.tellg();
	int* TourId = new int[ntours];
	string* TourName = new string[ntours];
	vector<int>* list_of_spots = new vector<int>[ntours];
	for (int i = 0; i < ntours; ++i) {
		int n;
		data >> TourId[i] >> TourName[i] >> n;
		vector<int> T;
		for (int k = 0; k < n; k++) {
			int sid;
			data >> sid;
			T.push_back(sid);
		}
		list_of_spots[i] = T;;
	}

	skipFileComments(&data);

	int** DistMatrix = new int* [nnodes];
	for (int i = 1; i < nnodes; ++i) {
		DistMatrix[i] = new int[nnodes];
		string buffer;
		data >> buffer;
		for (int j = 0; j < i; j++) 
			data >> DistMatrix[i][j];
	}

	data.close();

	//Now we create the instance
	vector<int> SelectedTours;
	for (int i = 0; i < ntours; ++i) SelectedTours.push_back(i);
	random_shuffle(SelectedTours.begin(), SelectedTours.end());
	SelectedTours.resize(Param->getParamIntValue(ParamCompObject::ntours));

	vector<int> SelectedHotels;
	while (SelectedHotels.size() < Param->getParamIntValue(ParamCompObject::ngroups)) {
		for (auto i : Hotels)
			for (int k = 0; k < Param->getParamIntValue(ParamCompObject::avrep); ++k)
				SelectedHotels.push_back(i);
	}
	random_shuffle(SelectedHotels.begin(), SelectedHotels.end());
	SelectedHotels.resize(Param->getParamIntValue(ParamCompObject::ngroups));

	set<int> InstanceNodes;
	for (auto i : SelectedHotels) InstanceNodes.insert(i);
	for (auto t : SelectedTours) for (auto i : list_of_spots[t])
		InstanceNodes.insert(i);

	string InstanceFile = Param->getParamValue(ParamCompObject::instance);
	string Folder = Param->getParamValue(ParamCompObject::folderpath);
	cout << "Generating Instance File " << InstanceFile << " ..." << endl;

	string filepath = Folder + InstanceFile;
	ofstream output(filepath.c_str());
	assert(output.is_open());

	int w = 15;
	int sw = 5;

	output << setw(sw) << "#" << setw(w) << "ngroups" << setw(w) << "ntours" << setw(w) << "nspots" << endl;
	output << setw(sw) << " " << setw(w) << Param->getParamIntValue(ParamCompObject::ngroups) << setw(w) << SelectedTours.size() << setw(w) << InstanceNodes.size() << endl;
	output << endl;

	output << setw(sw) << "#" << setw(w) << "spot_id" << setw(3*w) << "n_name" << setw(w) << "length" << setw(w) << "a" << setw(w) << "b" << endl;
	for (auto i : InstanceNodes)
		output << setw(sw) << " " << setw(w) << NodeId[i] << setw(3*w) << NodeName[i] << setw(w) << StayLength[i] << setw(w) << 0 << setw(w) << Param->getParamIntValue(ParamCompObject::planhor) << endl;
	output << endl;

	output << setw(sw) << "#" << setw(w) << "tr_id" << setw(3*w) << "tr_name" << setw(w) << "nspot" << setw(w) << "seq" << endl;

	for (auto t : SelectedTours) {
		output << setw(sw) << " " << setw(w) << TourId[t] << setw(3*w) << TourName[t] << setw(w) << list_of_spots[t].size();
		for (auto i : list_of_spots[t]) output << setw(w) << NodeId[i];
		output << endl;
	}
	output << endl;

	output << setw(sw) << "#" << setw(w) << "gr_id" << setw(w) << "gr_size" << setw(w) << "tr_id" << setw(w) << "origin" << endl;
	for (int g = 0; g < Param->getParamIntValue(ParamCompObject::ngroups); ++g) {
		int t = SelectedTours[g % SelectedTours.size()];
		if (g >= SelectedTours.size() - 1) random_shuffle(SelectedTours.begin(), SelectedTours.end());
		int gsize = RAND(Param->getParamIntValue(ParamCompObject::mingsize), Param->getParamIntValue(ParamCompObject::maxgsize));
		output << setw(sw) << " " << setw(w) << g << setw(w) << gsize << setw(w) << TourId[t] << setw(w) << SelectedHotels[g] << endl;
	}
	output << endl;

	output << setw(sw) << "#" << setw(w) << "o/d";
	for (auto i : InstanceNodes) output << setw(w) << NodeId[i]; output << endl;
	for (auto i : InstanceNodes) if(i!=*InstanceNodes.begin()){
		output << setw(sw) << " " << setw(w) << NodeId[i];
		for (auto j : InstanceNodes) if (j < i)
			output << setw(w) << DistMatrix[i][j];
		output << endl;
	}

	output.close();

	cout << "... " << InstanceFile << " created in Folder " << Folder << endl;

	//Cleaning
	delete[] NodeId;
	delete[] StayLength;
	delete[] NodeName;

	delete[] TourId;
	delete[] TourName;
	delete[] list_of_spots;

	for (int i = 1; i < nnodes; ++i) delete[] DistMatrix[i];
	delete[] DistMatrix;

}

void Instance::genRequiredFile(Parameter* Param)
{
	if (Param->hasDataFileToCreate()) genDataFile(Param);
	else if (Param->hasInstanceToCreate()) genInstanceFile(Param);
	else {
		cout << "There is no file to create... " << endl;
	}
}

Instance::Instance(Parameter* Param)
{
	resetallidgen();

	_Param = Param;
	srand(_Param->getParamIntValue(ParamCompObject::seed));
	_nspots = _nnodes = 0;
	_ngroups = 0;
	_vcapacity = 0;
	_nodes = NULL;
	_spots = NULL;
	_groups = NULL;
	_tours = NULL;
	_timematrix = NULL;
	_planhor = 0;

	assert(_Param->hasInstance());

	read();

	//Some post processing
	correctTimeMatrix();

	double alpha = _Param->getParamIntValue(ParamCompObject::QoS) / 100.0;
	for (int g = 0; g < _ngroups; ++g) {
		int i = firstgroupnode(g);
		int length = 0;
		do {
			int j = nextnodeid(i);
			length += travellingtime(i, j);
			i = j;
		} while (!islastnode(i));

		_groups[g]->_mintrlength = length;
		_groups[g]->_maxtrlength = planhor();
		if(alpha>0.001)
			_groups[g]->_maxtrlength = floor((double)length / alpha);
	}

	_LowerBound = 1;
}

Instance::~Instance()
{
	for (int i = 0; i < _nnodes; ++i)  delete _nodes[i];
	delete[] _nodes;

	for (int i = 0; i < _nspots; ++i) delete _spots[i];
	delete[] _spots;

	for (int t = 0; t < _ntours; ++t) delete _tours[t];
	delete[] _tours;


	for (int i = 0; i < _nspots; ++i) if (i != 0) delete[] _timematrix[i];
	delete[] _timematrix;

}

string Instance::instancename()
{
	string InstName = _Param->getParamValue(ParamCompObject::instance);
	size_t pos = InstName.find_last_of('/');
	pos = 0;
	return InstName.substr(pos);
}

int Instance::travellingtime(int o_nodeid, int d_nodeid)
{
	spot* O = _nodes[o_nodeid]->_spot;
	spot* D = _nodes[d_nodeid]->_spot;
	if (O == D && _nodes[o_nodeid]->_next == _nodes[d_nodeid]) {
		assert(isdeliverynode(o_nodeid) && ispickupnode(d_nodeid));
		return O->_lengthofstay;
	}
	else return travellingtime(O, D);
}

int Instance::travellingtime(spot* O, spot* D)
{
	int o = O->_id;
	int d = D->_id;
	if (o < d) return _timematrix[d][o];
	else if (o > d) return _timematrix[o][d];
	else return 0;
}

void Instance::print()
{
	int w = 10;
	string InstName = _Param->getParamValue(ParamCompObject::instance);
	size_t pos = InstName.find_last_of('/');
	pos = 0;
	cout << setw(10) << "Instance: " << setw(10) << InstName.substr(pos) << endl;

	cout << setw(10) << "Number of groups: " << setw(10) << _ngroups << endl;
	for (int g = 0; g < _ngroups; ++g) {
		cout << tostring(_groups[g]) << endl;
	}

	cout << setw(10) << "Number of tours: " << setw(10) << _ntours << endl;
	for (int t = 0; t < _ntours; ++t) {
		cout << tostring(_tours[t]) << endl;
	}

	cout << setw(10) << "Number of nodes: " << setw(10) << _nnodes << endl;
	for (int i = 0; i < _nnodes; ++i) {
		cout << tostring(_nodes[i]) << endl;
	}
}

TwoDimNode::TwoDimNode(const vector<TwoDimNode*>& Points, double mindist, double l) {
	_l = l;
	bool feas = false;
	while (!feas) {
		_x = RAND01();
		_y = RAND01();
		feas = true;
		for (auto P : Points) {
			if (distanceto(*P) < mindist - 0.001) {
				feas = false;
				break;
			}
		}
	}
	for (auto P1 : Points) {
		assert(this->distanceto(*P1) >= mindist);
		assert(P1->distanceto(*this) >= mindist);
	}
}

double TwoDimNode::length(const vector<TwoDimNode*>& Points) {
	double dist = 0.0;
	if (!Points.empty()) {
		TwoDimNode* P0 = Points.front();
		for (auto P : Points) {
			dist += P->_l + P0->distanceto(*P);
			P0 = P;
		}
	}
	return dist;
}

void TwoDimNode::print(const vector<TwoDimNode*>& Points) {
	int w = 10;
	if (Points.empty()) {
		cout << "Tour is Empty";
	}
	else {
		cout << " [" << setw(w) << to_string(Points.size()) + " / " + to_string((int)length(Points)) << " ]: ";
		TwoDimNode* P0 = Points.front();
		cout << "[" << to_string((int)P0->_l) << "]";
		for (auto P : Points) {
			if (P != P0) {
				double dist = P0->distanceto(*P);
				cout << "--" << (int)dist << "->" << "[" + to_string((int)P->_l) + "]";
			}
			P0 = P;
		}
	}
	cout << endl;
}
