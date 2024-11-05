#include "MST.hpp"
#include <queue>
#include <algorithm>
#include <limits>

using namespace std;

/**
 * MST Constructor
 * Initializes the MST based on the provided graph and the chosen algorithm (either "kruskal" or "boruvka").
 * The MST is represented as an adjacency matrix, initialized to zero.
 * If an invalid algorithm is specified, an error message is printed.
 *
 * @param graph The input graph represented by an adjacency matrix.
 * @param algo The chosen algorithm for constructing the MST.
 */
MST::MST(Graph graph, string algo) : graph(graph)
{
    vector<vector<int>> adjmat = graph.getAdjMat();
    int n = adjmat.size();
    if (n == 0)
    {
        mst = vector<vector<int>>();
    }
    mst = vector<vector<int>>(n, vector<int>(n, 0));

    if (algo == "kruskal")
    {
        kruskal(adjmat);
    }
    else if (algo == "boruvka")
    {
        boruvka(adjmat);
    }
    else
    {
        cout << "Invalid algorithm" << endl;
    }
}

/**
 * getWeight
 * Calculates and returns the total weight of the edges in the MST.
 *
 * @return The total weight of the MST, or 0 if the MST is empty.
 */
int MST::getWieght()
{
    if (mst.empty())
        return 0;
    int weight = 0;
    for (size_t i = 0; i < mst.size(); i++)
    {
        for (size_t j = i + 1; j < mst[i].size(); j++)
        {
            weight += mst[i][j];
        }
    }
    return weight;
}

/**
 * getMST
 * Returns the adjacency matrix of the MST.
 *
 * @return The MST adjacency matrix.
 */
vector<vector<int>> MST::getMST()
{
    return mst;
}

/**
 * kruskal
 * Constructs the MST using Kruskal's algorithm, which sorts all edges and
 * adds them incrementally while avoiding cycles.
 *
 * @param adj The adjacency matrix of the input graph.
 */
void MST::kruskal(vector<vector<int>> &adj)
{
    if (adj.empty())
        return;
    int n = adj.size();
    vector<tuple<int, int, int>> edges;

    // Collect all edges with weights into a list of tuples
    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            if (adj[i][j] > 0)
            {
                edges.emplace_back(adj[i][j], i, j);
            }
        }
    }

    sort(edges.begin(), edges.end());

    vector<int> parent(n);
    for (int i = 0; i < n; i++)
        parent[i] = i;

    function<int(int)> find = [&](int x)
    {
        if (parent[x] != x)
            parent[x] = find(parent[x]);
        return parent[x];
    };

    auto unite = [&](int x, int y)
    {
        x = find(x);
        y = find(y);
        if (x != y)
            parent[y] = x;
    };

    // Add edges to MST, avoiding cycles
    for (const auto &[w, u, v] : edges)
    {
        if (find(u) != find(v))
        {
            unite(u, v);
            mst[u][v] = mst[v][u] = w;
        }
    }
}

/**
 * boruvka
 * Constructs the MST using Borůvka's algorithm, which finds the cheapest edge
 * for each component and merges components until only one remains.
 *
 * @param adj The adjacency matrix of the input graph.
 */
void MST::boruvka(vector<vector<int>> &adj)
{
    if (adj.empty())
        return;
    int n = adj.size();
    vector<int> parent(n);
    vector<int> rank(n, 0);
    for (int i = 0; i < n; i++)
        parent[i] = i;

    function<int(int)> find = [&](int x)
    {
        if (parent[x] != x)
            parent[x] = find(parent[x]);
        return parent[x];
    };

    auto unite = [&](int x, int y)
    {
        x = find(x);
        y = find(y);
        if (x == y)
            return false;
        if (rank[x] < rank[y])
            swap(x, y);
        parent[y] = x;
        if (rank[x] == rank[y])
            rank[x]++;
        return true;
    };

    bool change = true;
    while (change)
    {
        change = false;
        vector<pair<int, int>> cheapest(n, {-1, -1});

        // Find the cheapest edge for each component
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (adj[i][j] > 0)
                {
                    int set1 = find(i), set2 = find(j);
                    if (set1 != set2)
                    {
                        if (cheapest[set1].second == -1 || adj[i][j] < adj[cheapest[set1].first][cheapest[set1].second])
                        {
                            cheapest[set1] = {i, j};
                        }
                        if (cheapest[set2].second == -1 || adj[i][j] < adj[cheapest[set2].first][cheapest[set2].second])
                        {
                            cheapest[set2] = {i, j};
                        }
                    }
                }
            }
        }

        // Add the cheapest edges to MST and merge components
        for (int i = 0; i < n; i++)
        {
            if (cheapest[i].second != -1)
            {
                int u = cheapest[i].first, v = cheapest[i].second;
                int set1 = find(u), set2 = find(v);
                if (set1 != set2)
                {
                    mst[u][v] = mst[v][u] = adj[u][v];
                    unite(set1, set2);
                    change = true;
                }
            }
        }
    }
}

/**
 * shortestPath
 * Finds the shortest path between two nodes in the MST using BFS.
 *
 * @param start The start node.
 * @param end The end node.
 * @return A vector representing the path from start to end, or an empty vector if no path exists.
 */
vector<int> MST::shortestPath(int start, int end)
{
    if (mst.empty())
        return {};
    
    if (start == end)
        return {start};
    
    int n = mst.size();
    vector<bool> visited(n, false);
    vector<int> parent(n, -1);
    queue<int> q;

    q.push(start);
    visited[start] = true;

    while (!q.empty())
    {
        int u = q.front();
        q.pop();

        if (u == end)
            break;

        for (int v = 0; v < n; v++)
        {
            if (mst[u][v] > 0 && !visited[v])
            {
                visited[v] = true;
                parent[v] = u;
                q.push(v);
            }
        }
    }

    if (!visited[end])
        return {};

    vector<int> path;
    for (int v = end; v != -1; v = parent[v])
    {
        path.push_back(v);
    }
    reverse(path.begin(), path.end());
    return path;
}

/**
 * longestPath
 * Finds the longest path between two nodes in the MST using DFS.
 *
 * @param start The start node.
 * @param end The end node.
 * @return A vector representing the longest path from start to end, or an empty vector if no path exists.
 */
vector<int> MST::longestPath(int start, int end)
{
    if (mst.empty())
        return {};
    int n = mst.size();
    vector<int> dist(n, -1);
    vector<int> parent(n, -1);

    function<void(int, int)> dfs = [&](int u, int p)
    {
        for (int v = 0; v < n; v++)
        {
            if (mst[u][v] > 0 && v != p)
            {
                dist[v] = dist[u] + 1;
                parent[v] = u;
                dfs(v, u);
            }
        }
    };

    dist[start] = 0;
    dfs(start, -1);

    if (dist[end] == -1)
        return {};

    vector<int> path;
    for (int v = end; v != -1; v = parent[v])
    {
        path.push_back(v);
    }
    reverse(path.begin(), path.end());
    return path;
}

/**
 * averageDist
 * Calculates the average distance of all edges in the MST.
 *
 * @return The average edge distance or -1 if no paths exist in the MST.
 */
int MST::averageDist() {
    if (mst.empty())
        return -1;

    int totalDistance = 0;
    int pathCount = 0;
    size_t n = mst.size();

    for (size_t start = 0; start < n; ++start) {
        for (size_t end = start + 1; end < n; ++end) {
            if (mst[start][end] > 0) {
                totalDistance += mst[start][end];
                pathCount++;
            }
        }
    }

    if (pathCount == 0)
        return -1;

    return (totalDistance + pathCount) / pathCount;
}
