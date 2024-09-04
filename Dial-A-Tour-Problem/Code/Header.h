//
//  Header.h
//  Heur_SFP
//
//  Created by Vincent on 11/6/15.
//
//

#ifndef Header_h
#define Header_h

#include <string>
#include <ilconcert/iloenv.h>
#include <ilcplex/ilocplex.h>
#include <ilcp/cp.h>

#include <set>
#include <list>
#include <vector>
#include <algorithm>

#include <iomanip>
#include <assert.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <sstream> 
#include <fstream>
#include <limits.h>
#include <map>

#include <chrono>

using namespace std;

#if defined(_WIN32) || defined(_WIN64)
#define PATHSEP "\\" // Windows File Path Separator
#else
#define PATHSEP "/" // Linux / OSX
#endif

#define RAND01() (((double)(rand()%10001))/(double)10000)
template<typename T1, typename T2>
constexpr auto RAND(T1 MIN, T2 MAX) { return ((MIN!=MAX)?(MIN)+(rand()%((MAX)-(MIN))):MIN); }

struct customcomp {
private:
    double *_w;
public:
    customcomp(double *w) {_w=w;}
    bool operator() (int i,int j) { return (_w[i]+0.0001<_w[j]);}
    double operator() (int i) { return _w[i];}
};


template <class T, class Alloc= allocator<T> >
T RouletteWheelSelection(Alloc setofitem, double* weigth){
    assert(weigth!=NULL);
    assert(!setofitem.empty());
    unsigned long n=setofitem.size();

    double sum=0.0;
    for(int i=0;i<n;i++) sum+=weigth[i];

    double r=sum*RAND01();

    T val=*setofitem.begin();
    sum=0.0;
    int k=0;
    for(auto v:setofitem){
        sum+=weigth[k];
        if(sum>=r){
            val=v;
            break;
        }
        k++;
    }

    return val;
}

template <class ForwardIterator, class Eval>
ForwardIterator RouletteWheelSelection (ForwardIterator first, ForwardIterator last, Eval eval){

    if (first==last) return last;

    double sum=0;
    ForwardIterator it=first;
    while (++it!=last) sum+=eval(*it);

    double r=sum*RAND01();

    sum=0.0;
    while (first!=last){
        sum+=eval(*first);
        if (sum>=r-0.001)  break;
        first++;
    }
    assert(first!=last);
    return first;
}

#endif /* Header_h */
