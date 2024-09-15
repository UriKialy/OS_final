#include "MST.hpp"
#include <queue>
#include <algorithm>
#include <limits>

using namespace std;

MST::MST(Graph graph, string algo) : graph(graph) {
    vector<vector<int>> adjmat = graph.getAdjMat();
    int n = adjmat.size();
    if(n==0){
        mst=vector<vector<int>>();
    } 
    mst = vector<vector<int>>(n, vector<int>(n, 0));

    if (algo == "prim") {
        prim(adjmat);
    } else if (algo == "kruskal") {
        kruskal(adjmat);
    } else if (algo == "boruvka") {
        boruvka(adjmat);
    } else if (algo == "tarjan") {
        tarjan(adjmat);
    } else if (algo == "integerMST") {
        integerMST(adjmat);
    } else {
        cout << "Invalid algorithm" << endl;
    }
}

int MST::getWieght() {
    if(mst.empty()) return 0;
    int weight = 0;
    for (size_t i = 0; i < mst.size(); i++) {
        for (size_t j = i + 1; j < mst[i].size(); j++) {
            weight += mst[i][j];
        }
    }
    return weight;
}

void MST::printMST() {
    for (const auto& row : mst) {
        for (int val : row) {
            cout << val << " | ";
        }
        cout << endl;
    }
}

vector<vector<int>> MST::getMST() {
    return mst;
}

void MST::prim(vector<vector<int>>& adj) {
    if(adj.empty()) return;
    int n = adj.size();
    vector<int> key(n, numeric_limits<int>::max());
    vector<bool> inMST(n, false);
    vector<int> parent(n, -1);

    key[0] = 0;
    parent[0] = -1;

    for (int count = 0; count < n - 1; count++) {
        int u = -1;
        int min_key = numeric_limits<int>::max();
        for (int v = 0; v < n; v++) {
            if (!inMST[v] && key[v] < min_key) {
                u = v;
                min_key = key[v];
            }
        }

        inMST[u] = true;

        for (int v = 0; v < n; v++) {
            if (adj[u][v] && !inMST[v] && adj[u][v] < key[v]) {
                parent[v] = u;
                key[v] = adj[u][v];
            }
        }
    }

    for (int i = 1; i < n; i++) {
        mst[parent[i]][i] = mst[i][parent[i]] = adj[i][parent[i]];
    }
}

void MST::kruskal(vector<vector<int>>& adj) {
    if(adj.empty()) return;
    int n = adj.size();
    vector<tuple<int, int, int>> edges;

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (adj[i][j] > 0) {
                edges.emplace_back(adj[i][j], i, j);
            }
        }
    }

    sort(edges.begin(), edges.end());

    vector<int> parent(n);
    for (int i = 0; i < n; i++) parent[i] = i;

    function<int(int)> find = [&](int x) {
        if (parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    };

    auto unite = [&](int x, int y) {
        x = find(x);
        y = find(y);
        if (x != y) parent[y] = x;
    };

    for (const auto& [w, u, v] : edges) {
        if (find(u) != find(v)) {
            unite(u, v);
            mst[u][v] = mst[v][u] = w;
        }
    }
}

void MST::boruvka(vector<vector<int>>& adj) {
    if(adj.empty()) return;
    int n = adj.size();
    vector<int> parent(n);
    vector<int> rank(n, 0);
    for (int i = 0; i < n; i++) parent[i] = i;

    function<int(int)> find = [&](int x) {
        if (parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    };

    auto unite = [&](int x, int y) {
        x = find(x);
        y = find(y);
        if (x == y) return false;
        if (rank[x] < rank[y]) swap(x, y);
        parent[y] = x;
        if (rank[x] == rank[y]) rank[x]++;
        return true;
    };

    bool change = true;
    while (change) {
        change = false;
        vector<pair<int, int>> cheapest(n, {-1, -1});

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (adj[i][j] > 0) {
                    int set1 = find(i), set2 = find(j);
                    if (set1 != set2) {
                        if (cheapest[set1].second == -1 || adj[i][j] < adj[cheapest[set1].first][cheapest[set1].second]) {
                            cheapest[set1] = {i, j};
                        }
                        if (cheapest[set2].second == -1 || adj[i][j] < adj[cheapest[set2].first][cheapest[set2].second]) {
                            cheapest[set2] = {i, j};
                        }
                    }
                }
            }
        }

        for (int i = 0; i < n; i++) {
            if (cheapest[i].second != -1) {
                int u = cheapest[i].first, v = cheapest[i].second;
                int set1 = find(u), set2 = find(v);
                if (set1 != set2) {
                    mst[u][v] = mst[v][u] = adj[u][v];
                    unite(set1, set2);
                    change = true;
                }
            }
        }
    }
}

void MST::tarjan(vector<vector<int>>& adj) {
    if(adj.empty()) return;
    int n = adj.size();
    vector<int> parent(n);
    vector<int> rank(n, 0);
    for (int i = 0; i < n; i++) parent[i] = i;

    function<int(int)> find = [&](int x) {
        if (parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    };

    auto unite = [&](int x, int y) {
        x = find(x);
        y = find(y);
        if (x == y) return false;
        if (rank[x] < rank[y]) swap(x, y);
        parent[y] = x;
        if (rank[x] == rank[y]) rank[x]++;
        return true;
    };

    vector<tuple<int, int, int>> edges;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (adj[i][j] > 0) {
                edges.emplace_back(adj[i][j], i, j);
            }
        }
    }

    sort(edges.begin(), edges.end());

    for (const auto& [w, u, v] : edges) {
        if (unite(u, v)) {
            mst[u][v] = mst[v][u] = w;
        }
    }
}



void MST::integerMST( vector<vector<int>>& adj) {
    int n = adj.size();
    
    // Initialize parent and rank arrays for union-find
    vector<int> parent(n), rank(n, 0);
    
    // Union-Find 'find' function with path compression
    function<int(int)> find = [&](int x) {
        if (parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    };

    // Union-Find 'unite' function with union by rank
    auto unite = [&](int x, int y) {
        int px = find(x), py = find(y);
        if (px == py) return;
        if (rank[px] < rank[py]) swap(px, py);
        parent[py] = px;
        if (rank[px] == rank[py]) rank[px]++;
    };

    // Initialize the parent array where each node is its own parent
    for (int i = 0; i < n; i++) parent[i] = i;
    


    // Number of components initially is equal to number of nodes
    int components = n;

    while (components > 1) {
        // Store the cheapest edge for each component
        vector<int> cheapest_edge(n, -1);

        // Find the cheapest edge for each component
        for (int u = 0; u < n; ++u) {
            for (int v = 0; v < n; ++v) {
                if (adj[u][v] != 0 && find(u) != find(v)) { // valid edge between different components
                    int pu = find(u);
                    if (cheapest_edge[pu] == -1 || adj[u][v] < adj[u][cheapest_edge[pu]]) {
                        cheapest_edge[pu] = v;
                    }
                }
            }
        }

        // Union components using the cheapest edges
        for (int u = 0; u < n; ++u) {
            int v = cheapest_edge[u];
            if (v != -1 && find(u) != find(v)) {
                // Add this edge to the MST
                mst.push_back({u, v, adj[u][v]});
                unite(u, v);
                components--; // One less component
            }
        }

        // If no new edges were found, we break the loop
        if (all_of(cheapest_edge.begin(), cheapest_edge.end(), [](int x) { return x == -1; })) {
            break;
        }
    }

}


vector<int> MST::shortestPath(int start, int end) {
    if(mst.empty()) return {};
    if(start==end) return {start};
    int n = mst.size();
    vector<bool> visited(n, false);
    vector<int> parent(n, -1);
    queue<int> q;

    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        if (u == end) break;

        for (int v = 0; v < n; v++) {
            if (mst[u][v] > 0 && !visited[v]) {
                visited[v] = true;
                parent[v] = u;
                q.push(v);
            }
        }
    }

    if (!visited[end]) return {};

    vector<int> path;
    for (int v = end; v != -1; v = parent[v]) {
        path.push_back(v);
    }
    reverse(path.begin(), path.end());
    return path;
}

vector<int> MST::longestPath(int start, int end) {
    if(mst.empty()) return {};
    int n = mst.size();
    vector<int> dist(n, -1);
    vector<int> parent(n, -1);

    function<void(int, int)> dfs = [&](int u, int p) {
        for (int v = 0; v < n; v++) {
            if (mst[u][v] > 0 && v != p) {
                dist[v] = dist[u] + 1;
                parent[v] = u;
                dfs(v, u);
            }
        }
    };

    dist[start] = 0;
    dfs(start, -1);

    if (dist[end] == -1) return {};

    vector<int> path;
    for (int v = end; v != -1; v = parent[v]) {
        path.push_back(v);
    }
    reverse(path.begin(), path.end());
    return path;
}

int MST::averageDist(int start, int end) {
    if (mst.empty()) return -1;
    vector<int> path = shortestPath(start, end);
    if (path.empty()) return -1;
    return (path.size() - 1 + 1) / 2;  // Average rounded up
}