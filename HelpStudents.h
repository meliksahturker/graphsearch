#ifndef CMPE250_ASSIGNMENT3_HELPSTUDENTS_H
#define CMPE250_ASSIGNMENT3_HELPSTUDENTS_H
#include <vector>
#include <iostream>
#include <fstream>
#include <list>
#include<limits>
#include <queue>

using namespace std;

const long long int INF = numeric_limits<long long int>::max();

class HelpStudents{

public:
    HelpStudents(int  N, int  M, int K, vector < pair< pair <int,int> , int > > ways);
    long long int firstStudent();
    long long int secondStudent();
    long long int thirdStudent();
    long long int fourthStudent();
    long long int fifthStudent();
    // YOU CAN ADD YOUR HELPER FUNCTIONS AND MEMBER FIELDS

};

// Dijkstra
class Dijkstra {

    const long long int INF = numeric_limits<long long int>::max();
private:
    int numberOfVertices;
    list< pair<long long int, long long int> > *adjacencyList;

public:
    Dijkstra(int numberOfVertices);

    ~Dijkstra() {
        delete[] adjacencyList;
    }
    void addEdge(long long int u,long long int v, long long int w);
    vector<long long int> shortestPath(int s);
};

// Kruskal's:
#define edge pair<int,int>
class Kruskal {
private:
    vector<pair<int, edge>> G; // graph
    vector<pair<int, edge>> T; // mst
    int *parent;
    int V;
public:
    Kruskal(int V);
    void AddWeightedEdge(int u, int v, int w);
    int find_set(int i);
    void union_set(int u, int v);
    void kruskal();
    vector<pair<int, edge>> print();
};

class Greed{
public:
    vector < pair <int, int >> vec_four_all[100000];
    void recursiveGreedFunction(int u);
};



#endif //CMPE250_ASSIGNMENT3_HELPSTUDENTS_H
