/*
Student Name: Meliksah Turker
Student Number: 2019700120
Project Number: 4
Compile Status: [SUCCESS]
Running Status: [SUCCESS]
Notes: I used Dijkstra's algorithm for student 1 and 3. Student 3's case is simply the one with all edge weights equal to 1.
Student 2's case was a MST problem where I first discovered the graph using Dijkstra's algorithm whether there are disconnected parts.
Then I removed the parts that are not connected to source and target, built a MST using Kruskal's algorithm. THen searched this tree using BFS and saved the path
from source to target. And saving the weights of this path, I later summed them to obtain final result for Student 2.
I implemented Student 4's Greedy algorithm however the trick was to remove the edges that I travelled. In order to be able to do that, I saved the index of the edge
in adjacency vector but then trick was again to remove both (u,v) and (v,u). I used an O(logN) for loop to find corresponding edge and removed it, too.
I did not implemented Student 5 due to time constraints caused by CmpE 322 midterm.
*/
#include "HelpStudents.h"
#include <list>
#include <queue>
#include<set>
#include<vector>
#include <climits>
#include<iostream>
#include<limits>
#include<algorithm>
#include <bits/stdc++.h>


using namespace std;

int V, E, target, no_of_connected_edges = 0, max_for_second = 0;;
vector<int> path;
long long int sp = 0, a, b, c, sum_of_four = 0 ;
vector < pair < pair <long long int, long long int> , long long int >> edges;
vector < pair <int, int >> tempo_for_four;


HelpStudents::HelpStudents(int  N, int  M, int K, vector < pair< pair <int,int> , int > > ways) {
    V = N;
    E = M;
    target = K;
    for (int i = 0; i< ways.size(); i++){
        a = (long long) ways[i].first.first;
        b = (long long) ways[i].first.second;
        c = (long long) ways[i].second;
        edges.push_back(make_pair(make_pair(a,b), c));
    }
}



// Path Printer Helper Function for Second Student
void add_edge(vector <pair<int, int>> adj[], int src, int dest, int w)
{
    adj[src].push_back(make_pair(dest, w));
    adj[dest].push_back(make_pair(src, w));
}

bool BFS(vector<pair <int, int>> adj[], int src, int dest, int v,
         int pred[], int dist[])
{

    list<int> queue;

    bool visited[v];

    for (int i = 0; i < v; i++) {
        visited[i] = false;
        dist[i] = INT_MAX;
        pred[i] = -1;
    }

    visited[src] = true;
    dist[src] = 0;
    queue.push_back(src);

    // standard BFS algorithm
    while (!queue.empty()) {
        int u = queue.front();
        queue.pop_front();
        for (int i = 0; i < adj[u].size(); i++) {
            if (visited[adj[u][i].first] == false) {
                visited[adj[u][i].first] = true;
                dist[adj[u][i].first] = dist[u] + 1;
                pred[adj[u][i].first] = u;
                queue.push_back(adj[u][i].first);


                if (adj[u][i].first == dest)
                    return true;
            }
        }
    }

    return false;
}


void printShortestDistance(vector<pair <int, int>> adj[], int s,
                           int dest, int v)
{
    int pred[v], dist[v];

    if (BFS(adj, s, dest, v, pred, dist) == false)
    {
        return;
    }

    int crawl = dest;
    path.push_back(crawl);
    while (pred[crawl] != -1) {
        path.push_back(pred[crawl]);
        crawl = pred[crawl];
    }

    for (int i = path.size() - 1; i >= 0; i--){
        for (int j = 0 ; j < adj[path[i]].size(); j++){
            if (adj[path[i]][j].first == path[i-1]){
                if ( adj[path[i]][j].second > max_for_second){
                    max_for_second = adj[path[i]][j].second;
                }
            }
        }
    }
}





long long int HelpStudents::firstStudent() {    // Shortest Path
    Dijkstra dijkstra(V);
    for (int i = 0; i < E; i++){
        dijkstra.addEdge(edges[i].first.first-1, edges[i].first.second-1, edges[i].second);
    }
    dijkstra.shortestPath(0);
    return sp;
}
long long int HelpStudents::secondStudent() {
    Dijkstra dijkstra(V);
    for (int i = 0; i< E; i++) {
        int u = edges[i].first.first - 1;
        int v = edges[i].first.second - 1;
        int w = edges[i].second;
        dijkstra.addEdge(u, v, w);
    }
    vector<long long int> tempo_dist = dijkstra.shortestPath(0);

    Kruskal k(V);
    for (int i = 0; i< E; i++) {
        int u = edges[i].first.first - 1;
        int v = edges[i].first.second - 1;
        int w = edges[i].second;

        if (tempo_dist[u] != INF){
            k.AddWeightedEdge(u, v, w);
        }
    }

    k.kruskal();

    vector<pair<int, edge>> MST;
    MST = k.print();

    int v = V;
    vector <pair <int, int >> adj[V];
    for (int i= 0; i < no_of_connected_edges ; i++){
        int tempo_v = MST[i].second.first;
        int tempo_u = MST[i].second.second;
        int tempo_w = MST[i].first;
        add_edge(adj, tempo_v, tempo_u, tempo_w);
    }

    printShortestDistance(adj, 0, target-1, v);
    return max_for_second;
}



void Greed::recursiveGreedFunction(int u){

    if (vec_four_all[u].size() > 0){

        struct Vec4{
            int x, y, z;
        };
        std::vector< Vec4> v;
        for (int i = 0; i < vec_four_all[u].size(); i++) {
            Vec4 vec4;
            vec4.x = vec_four_all[u][i].second; // u
            vec4.y = vec_four_all[u][i].first; // w
            vec4.z = i; // index in the vector
            v.push_back(vec4);

        }
        std::sort(begin(v), end(v),
              [&](const auto &lhs, const auto &rhs)
              {
                  if(lhs.x<rhs.x) return true;
                  if(lhs.x>rhs.x) return false;
                  if(lhs.y<rhs.y) return true;
                  if(lhs.y>rhs.y) return false;
                  return lhs.z<rhs.z;
              });
            // increase sum_of_four
            // remove walked edge from the vector
            // call the recursive function
        sum_of_four += v[0].x;
        vec_four_all[u].erase(vec_four_all[u].begin() + v[0].z);
        int index_to_remove;
        for(int j = 0; j < vec_four_all[v[0].y].size(); j++){
            if (vec_four_all[v[0].y][j].first == u){
                index_to_remove = j;
                break;
            }
        }
        vec_four_all[v[0].y].erase(vec_four_all[v[0].y].begin() + index_to_remove);

        if (v[0].y != target -1){
            recursiveGreedFunction(v[0].y);
        }

    }
};





long long int HelpStudents::thirdStudent() {// minimum number of edges.
    Dijkstra dijkstra(V);
    for (int i = 0; i < E; i++){
        dijkstra.addEdge(edges[i].first.first-1, edges[i].first.second-1, 1);
    }
    dijkstra.shortestPath(0);

    return sp;
}

long long int HelpStudents::fourthStudent() {
    Greed g;
    for (int i = 0; i < E ; i ++){
        int u = edges[i].first.first-1;
        int v = edges[i].first.second-1;
        int w = edges[i].second;
        g.vec_four_all[u].push_back(make_pair(v,w));
        g.vec_four_all[v].push_back(make_pair(u,w));
    }

    g.recursiveGreedFunction(0);
    //cout << sum_of_four << endl;

    return sum_of_four;

}
long long int HelpStudents::fifthStudent() {
    // IMPLEMENT ME!
}



// Dijkstra's Algorithm
Dijkstra::Dijkstra(int numberOfVertices)
{
    this->numberOfVertices = numberOfVertices;
    adjacencyList = new list< pair<long long int, long long int>> [V];
}

void Dijkstra::addEdge(long long int u, long long int v, long long  int w) {
    adjacencyList[u].push_back(make_pair(v, w));
    adjacencyList[v].push_back(make_pair(u, w));
}

vector<long long int> Dijkstra::shortestPath(int src) {
    set< pair<long long int, long long int> > setds;
    vector<long long int> dist(numberOfVertices, INF);
    setds.insert(make_pair(0, src));
    dist[src] = 0;

    while (!setds.empty()) {

        pair<long long int, long long int> tmp = *(setds.begin());
        setds.erase(setds.begin());
        long long int u = tmp.second;

        list< pair<long long int, long long int> >::iterator i;
        for (i = adjacencyList[u].begin(); i != adjacencyList[u].end(); ++i)
        {
            long long int v = (*i).first;
            long long int weight = (long long) (*i).second;

            if ((long long) dist[v] > (long long) dist[u] + (long long) weight)
            {
                if (dist[v] != INF)
                    setds.erase(setds.find(make_pair(dist[v], v)));

                dist[v] = dist[u] + weight;
                setds.insert(make_pair(dist[v], v));
            }
        }
    }
    sp = dist[target-1];
    return dist;
}




// Kruskal's Algorithm

Kruskal::Kruskal(int V) {
    parent = new int[V];

    for (int i = 0; i < V; i++)
        parent[i] = i;

    G.clear();
    T.clear();
}
void Kruskal::AddWeightedEdge(int u, int v, int w) {
    G.push_back(make_pair(w, edge(u, v)));
}
int Kruskal::find_set(int i) {
    if (i == parent[i])
        return i;
    else
        return find_set(parent[i]);
}

void Kruskal::union_set(int u, int v) {
    parent[u] = parent[v];
}
void Kruskal::kruskal() {
    int i, uRep, vRep;
    sort(G.begin(), G.end()); // increasing weight
    for (i = 0; i < G.size(); i++) {
        uRep = find_set(G[i].second.first);
        vRep = find_set(G[i].second.second);
        if (uRep != vRep) {
            T.push_back(G[i]); // add to tree
            union_set(uRep, vRep);
        }
    }
}
vector<pair<int, edge>> Kruskal::print() {
    no_of_connected_edges = T.size();
    return T;
}

