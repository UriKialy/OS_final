#include "Graph.hpp"

using namespace std;
class MST
{
    Graph graph;
    vector<vector<int>> mst;

public:
    MST(Graph graph, string algo);
    int getWieght();
    int longestDist();
    int averageDist();
    int shortestPath(int start, int end);
    vector<vector<int>> prim(vector<vector<int>> &adj);
    vector<vector<int>> kruskal(vector<vector<int>> &adj);
    vector<vector<int>> boruvka(vector<vector<int>>& adj) ;
    vector<vector<int>> tarjan(vector<vector<int>>& adj);

};