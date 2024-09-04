#include "Graph.hpp"

Graph::Graph(vector<vector<int>> adjMat): adjMat(adjMat) {
    numVertices = adjMat.size();
    numEdges = 0;
    for (int i = 0; i < numVertices; i++) {
        for(int j=0;j<numVertices;j++){
            if(adjMat.at(i).at(j)!=0){
            numEdges++;
        }
    }
}
}
int Graph::getNumEdges(){
    return numEdges;
}
int Graph::getNumVertices(){
    return numVertices;
}
vector<vector<int>> Graph::getAdjMat(){
    return adjMat;
}