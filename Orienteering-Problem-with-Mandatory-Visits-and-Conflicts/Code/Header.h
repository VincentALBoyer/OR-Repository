#pragma once

#include <string>
#include <ilconcert/iloenv.h>
#include <ilcplex/ilocplex.h>
#include <ilcp/cp.h>


#include <set>
#include <unordered_set>
#include <list>
#include <stack>
#include <vector>
#include <algorithm>

#include <iomanip>
#include <assert.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <cfloat>
#include <limits.h>
#include <map>

#include <chrono>
#include <numeric>
#include <functional>
#include <random>
#include <filesystem>



using namespace std;


constexpr double EPS = 0.001;

#define RAND01() (((double)(rand()%10001))/(double)10000)
template<typename T1, typename T2>
constexpr auto RAND(T1 MIN, T2 MAX) { return ((MIN != MAX) ? (MIN)+(rand() % ((MAX)-(MIN))) : MIN); }

struct customcomp {
private:
    double* _w;
public:
    customcomp(double* w) { _w = w; }
    bool operator() (int i, int j) { return (_w[i] + 0.0001 < _w[j]); }
    double operator() (int i) { return _w[i]; }

};

template <typename T>
struct idcomp {
private:
    T* _w;
public:
    idcomp(T* w) { _w = w; }
    bool operator() (int i, int j) { return ((*_w)[i] < (*_w)[j]); }
    double operator() (int i) { return (*_w)[i]; }
};



template <class T, class Alloc = allocator<T> >
T RouletteWheelSelection(Alloc setofitem, double* weigth) {
    assert(weigth != NULL);
    assert(!setofitem.empty());
    unsigned long n = setofitem.size();

    if (n == 1) return *setofitem.begin();

    double sum = 0.0;
    for (unsigned long i = 0; i < n; i++) sum += weigth[i];

    double r = sum * RAND01();

    T val = *setofitem.begin();
    sum = 0.0;
    int k = 0;
    for (auto v : setofitem) {
        sum += weigth[k];
        if (sum >= r) {
            val = v;
            break;
        }
        k++;
    }


    return val;
}

template <class ForwardIterator, class Eval>
ForwardIterator RouletteWheelSelection(ForwardIterator first, ForwardIterator last, Eval eval) {

    if (first == last) return last;

    double sum = 0.0;
    for(ForwardIterator it=first;it!=last;++it) sum += eval(*it);

    // Generate a random value between 0 and sum
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, sum);
    double r = dis(gen);

    sum = 0.0;
    while (first != last) {
        sum += eval(*first);
        if (sum >= r )  break;
        first++;
    }
    assert(first != last);
    return first;

    //double sum = 0.0;
    //ForwardIterator it = first;
    //while (++it != last) sum += eval(*it);

    //double r = sum * RAND01();

    //sum = 0.0;
    //while (first != last) {
    //    sum += eval(*first);
    //    if (sum >= r )  break;
    //    first++;
    //}
    //assert(first != last);
    //return first;


}

// Define a hash function for a vector of ints
struct VectorHash {
    size_t operator()(const std::vector<int>& v) const {
        std::hash<int> hasher;
        size_t seed = 0;
        for (int i : v) {
            seed ^= hasher(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

// Define an equality function for vectors of ints
struct VectorEqual {
    bool operator()(const std::vector<int>& a, const std::vector<int>& b) const {
        return a == b;
    }
};

// Define a hash function for a pair of vectors
struct PairHash {
    size_t operator()(const std::pair<std::vector<int>, std::vector<int>>& pair) const {
        size_t hashValue = 0;
        for (int num : pair.first) {
            hashValue ^= std::hash<int>()(num) + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2);
        }
        for (int num : pair.second) {
            hashValue ^= std::hash<int>()(num) + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2);
        }
        return hashValue;
    }
};

// Define an equality function for a pair of vectors
struct PairEqual {
    bool operator()(const std::pair<std::vector<int>, std::vector<int>>& lhs,
        const std::pair<std::vector<int>, std::vector<int>>& rhs) const {
        return lhs.first == rhs.first && lhs.second == rhs.second;
    }
};
