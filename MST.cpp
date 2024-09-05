#include "MST.hpp"
#include <limits>
#include <functional>
#include <queue>
#include <stack>
#include <algorithm> 
#include <random>
#include <ctime>
using namespace std;

MST::MST(Graph graph, string algo) : graph(graph)
{
    vector<vector<int>> adjmat = graph.getAdjMat();
    if (algo == "prim")
    {
        mst = prim(adjmat);
    }
    else if (algo == "kruskal")
    {
        mst = kruskal(adjmat);
    }
    else if (algo == "boruvka")
    {
        mst = boruvka(adjmat);
    }
    else if (algo == "tarjan")
    {
        mst = tarjan(adjmat);
    }
    else if (algo == "integerMST")
    {
        mst = integerMST(adjmat);
    }
    else
    {
        cout << "Invalid algorithm" << endl;
    }
}

int MST::getWieght()
{
    int weight = 0;
    for (int i = 0; i < mst.size(); i++)
    {
        for (int j = 0; j < mst.size(); j++)
        {
            weight += mst.at(i).at(j);
        }
    }
    return weight;
}
vector<vector<int>> prim(vector<vector<int>> &adj)
{
    int n = adj.size();
    vector<vector<int>> spanning_tree;
    vector<int> min_weight(n, numeric_limits<int>::max());
    vector<int> parent(n, -1);
    vector<bool> in_mst(n, false);

    // Priority queue to store vertices that are being proccessed
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    // Start with vertex 0
    pq.push({0, 0});
    min_weight[0] = 0;

    while (!pq.empty())
    {
        int u = pq.top().second;
        pq.pop();

        if (in_mst[u])
            continue;

        in_mst[u] = true;

        // Add edge to spanning tree if it's not the starting vertex
        if (parent[u] != -1)
        {
            spanning_tree.push_back({parent[u], u, adj[parent[u]][u]});
        }

        // Check all adjacent vertices of u
        for (int v = 0; v < n; v++)
        {
            // If v is not in MST and weight of (u,v) is smaller than current weight of v
            if (!in_mst[v] && adj[u][v] != 0 && adj[u][v] < min_weight[v])
            {
                parent[v] = u;
                min_weight[v] = adj[u][v];
                pq.push({min_weight[v], v});
            }
        }
    }
    return spanning_tree;
}

vector<vector<int>> kruskal(vector<vector<int>> &adj)
{
    int n = adj.size();
    vector<vector<int>> edges;
    vector<vector<int>> spanning_tree;

    // Convert adjacency matrix to edge list
    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            if (adj[i][j] > 0)
            {
                edges.push_back({i, j, adj[i][j]});
            }
        }
    }

    // Sort edges by weight
    sort(edges.begin(), edges.end(),
         [](const vector<int> &a, const vector<int> &b)
         {
             return a[2] < b[2];
         });

    UnionFind uf(n);

    for (const auto &edge : edges)
    {
        int from = edge[0], to = edge[1], weight = edge[2];
        if (uf.unite(from, to))
        {
            spanning_tree.push_back({from, to, weight});
        }
    }

    return spanning_tree;
}
class UnionFind
{
private:
    vector<int> parent, rank;

public:
    UnionFind(int n) : parent(n), rank(n, 0)
    {
        for (int i = 0; i < n; i++)
            parent[i] = i;
    }

    int find(int x)
    {
        if (parent[x] != x)
            parent[x] = find(parent[x]);
        return parent[x];
    }

    bool unite(int x, int y)
    {
        int px = find(x), py = find(y);
        if (px == py)
            return false;
        if (rank[px] < rank[py])
            swap(px, py);
        parent[py] = px;
        if (rank[px] == rank[py])
            rank[px]++;
        return true;
    }
};

vector<vector<int>> MST::boruvka(vector<vector<int>>& adj) {
    int n = adj.size();
    vector<int> component(n, -1);
    vector<vector<int>> cheapest(n, vector<int>(3, -1));
    vector<vector<int>> ans;
    vector<vector<int>> selected_graph(n);
    int graph_cc = n;

    auto find_cheapest = [&](int v) {
        int min_cost = numeric_limits<int>::max();
        vector<int> min_edge(3, -1);
        for (int u = 0; u < n; ++u) {
            if (adj[v][u] > 0 && adj[v][u] < min_cost) {
                min_cost = adj[v][u];
                min_edge = {v, u, adj[v][u]};
            }
        }
        return min_edge;
    };

    while (graph_cc > 1) {
        fill(cheapest.begin(), cheapest.end(), vector<int>(3, -1));
        fill(component.begin(), component.end(), -1);
        int cur_cc = 0;

        function<void(int, int)> explore = [&](int root, int cc) {
            component[root] = cc;
            for (const auto& viz : selected_graph[root]) {
                if (component[viz] == -1) explore(viz, cc);
            }
        };

        for (int i = 0; i < n; ++i) {
            if (component[i] == -1) {
                explore(i, cur_cc);
                cur_cc++;
            }
        }

        for (int i = 0; i < n; ++i) {
            vector<int> edge = find_cheapest(i);
            if (edge[2] != -1) {
                int from = component[edge[0]], to = component[edge[1]];
                if (from == to) continue;
                if (cheapest[from][2] == -1 || edge[2] < cheapest[from][2])
                    cheapest[from] = edge;
                if (cheapest[to][2] == -1 || edge[2] < cheapest[to][2])
                    cheapest[to] = edge;
            }
        }

        for (int i = 0; i < n; ++i) {
            if (cheapest[i][2] == -1) continue;
            int from = cheapest[i][0], to = cheapest[i][1], cost = cheapest[i][2];
            if (component[from] != component[to]) {
                selected_graph[from].push_back(to);
                selected_graph[to].push_back(from);
                ans.push_back({from, to, cost});
            }
        }

        graph_cc = cur_cc;
    }

    return ans;
}

class TarjanSCC {
private:
    vector<vector<int>>& adj;
    vector<int> disc, low, stackMember;
    vector<vector<int>> SCCs;
    stack<int> st;
    int time;

    void SCCUtil(int u) {
        disc[u] = low[u] = ++time;
        st.push(u);
        stackMember[u] = 1;

        for (int v = 0; v < adj.size(); v++) {
            if (adj[u][v] > 0) {  // If there's an edge from u to v
                if (disc[v] == -1) {
                    SCCUtil(v);
                    low[u] = min(low[u], low[v]);
                }
                else if (stackMember[v] == 1) {
                    low[u] = min(low[u], disc[v]);
                }
            }
        }

        int w = 0;
        if (low[u] == disc[u]) {
            vector<int> component;
            while (st.top() != u) {
                w = st.top();
                component.push_back(w);
                stackMember[w] = 0;
                st.pop();
            }
            w = st.top();
            component.push_back(w);
            stackMember[w] = 0;
            st.pop();
            SCCs.push_back(component);
        }
    }

public:
    TarjanSCC(vector<vector<int>>& graph) : adj(graph) {}

    vector<vector<int>> findSCCs() {
        int V = adj.size();
        disc.assign(V, -1);
        low.assign(V, -1);
        stackMember.assign(V, 0);
        time = 0;

        for (int i = 0; i < V; i++) {
            if (disc[i] == -1) {
                SCCUtil(i);
            }
        }

        return SCCs;
    }
};

vector<vector<int>> tarjan(vector<vector<int>>& adj) {
    TarjanSCC tarjan(adj);
    return tarjan.findSCCs();
}

class DisjointSet {
private:
    vector<int> parent, rank;

public:
    DisjointSet(int n) : parent(n), rank(n, 0) {
        for (int i = 0; i < n; i++) parent[i] = i;
    }

    int find(int x) {
        if (parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    }

    void unite(int x, int y) {
        int px = find(x), py = find(y);
        if (px == py) return;
        if (rank[px] < rank[py]) swap(px, py);
        parent[py] = px;
        if (rank[px] == rank[py]) rank[px]++;
    }
};

vector<vector<int>> boruvkaStep(vector<vector<int>>& adj, DisjointSet& ds) {
    int n = adj.size();
    vector<vector<int>> cheapest(n, vector<int>(3, -1));
    vector<vector<int>> mst_edges;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (adj[i][j] > 0) {
                int set1 = ds.find(i), set2 = ds.find(j);
                if (set1 != set2) {
                    if (cheapest[set1][2] == -1 || adj[i][j] < cheapest[set1][2])
                        cheapest[set1] = {i, j, adj[i][j]};
                    if (cheapest[set2][2] == -1 || adj[i][j] < cheapest[set2][2])
                        cheapest[set2] = {i, j, adj[i][j]};
                }
            }
        }
    }

    for (int i = 0; i < n; i++) {
        if (cheapest[i][2] != -1) {
            int set1 = ds.find(cheapest[i][0]), set2 = ds.find(cheapest[i][1]);
            if (set1 != set2) {
                mst_edges.push_back(cheapest[i]);
                ds.unite(set1, set2);
            }
        }
    }

    return mst_edges;
}

vector<vector<int>> integerMST(vector<vector<int>>& adj) {
    int n = adj.size();
    DisjointSet ds(n);
    vector<vector<int>> mst;

    // Main loop of the algorithm
    while (true) {
        auto new_edges = boruvkaStep(adj, ds);
        if (new_edges.empty()) break;
        mst.insert(mst.end(), new_edges.begin(), new_edges.end());
    }

    return mst;
}