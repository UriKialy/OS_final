#include "Graph.hpp"
#include <limits>
#include <functional>
#include <queue>
#include <stack>
#include <algorithm> 
#include <random>
#include <ctime>
 #include <vector>
using namespace std;
class MST
{
    Graph graph;
    vector<vector<int>> mst;

public:
    MST(Graph graph, string algo);
    int getWieght();
    int averageDist(int start, int end);
    vector<int>longestPath(int start, int end);
    vector<int> shortestPath(int start, int end);
    vector<vector<int>> prim(vector<vector<int>> &adj);
    vector<vector<int>> kruskal(vector<vector<int>> &adj);
    vector<vector<int>> boruvka(vector<vector<int>>& adj) ;
    vector<vector<int>> tarjan(vector<vector<int>>& adj);
    vector<vector<int>> MST::integerMST(vector<vector<int>>& adj);
    vector<int> reconstructPath(const vector<int>& parent_node, int start, int end);
    void dfs(const vector<vector<pair<int, int>>>& adj, int node, int parent,vector<int>& distance, vector<int>& parent_node);
    vector<vector<pair<int, int>>> buildAdjList(const vector<vector<int>>& mst);

};