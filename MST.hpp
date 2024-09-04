
#include "Graph.hpp"

using namespace std;
class MST {
    Graph graph;
    vector<vector<int>> mst;
   public:
   MST(Graph graph);
   int getWieght();
   int longestDist();
   int averageDist(); 
   int shortestPath(int start, int end);


};