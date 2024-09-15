#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "MST.hpp"

class TestGraph
{
public:
    static Graph createSampleGraph()
    {
        vector<vector<int>> adj_matrix = {
            {0, 2, 0, 6, 0},
            {2, 0, 3, 8, 5},
            {0, 3, 0, 0, 7},
            {6, 8, 0, 0, 9},
            {0, 5, 7, 9, 0}};
        return Graph(adj_matrix);
    }
};

TEST_CASE("MST Algorithms")
{
    Graph graph = TestGraph::createSampleGraph();


    SUBCASE("Kruskal's Algorithm")
    {
        MST mst(graph, "kruskal");
       CHECK(mst.getWieght() == 16);
        std::cout<<"kruskal worked"<<std::endl;
    }

    SUBCASE("Boruvka's Algorithm")
    {
        MST mst(graph, "boruvka");
        CHECK(mst.getWieght() == 16);
        std::cout<<"boruvka worked"<<std::endl;
    }

    

    SUBCASE("Invalid Algorithm")
    {
        std::ostringstream oss;
        auto cout_buff = std::cout.rdbuf(oss.rdbuf());

        MST mst(graph, "invalid");

        std::cout.rdbuf(cout_buff);
        CHECK(oss.str() == "Invalid algorithm\n");
    }
}

TEST_CASE("MST Path Finding")
{
    Graph graph = TestGraph::createSampleGraph();
    MST mst(graph, "prim");

    SUBCASE("Shortest Path")
    {
        vector<int> path = mst.shortestPath(0, 4);
        vector<int> expected_path = {0, 1, 4};
        CHECK(path == expected_path);
    }

    SUBCASE("Longest Path")
    {
        vector<int> path = mst.longestPath(0, 4);
        CHECK(path.front() == 0);
        CHECK(path.back() == 4);
        CHECK(path.size() >= 3); // The path should have at least 3 nodes

        // Check if the path is valid in the MST
        for (size_t i = 0; i < path.size() - 1; ++i)
        {
            CHECK(mst.getMST()[path[i]][path[i + 1]] > 0);
        }
    }

    SUBCASE("Average Distance")
    {
        int avg_dist = mst.averageDist();
        CHECK(avg_dist == 2);
    }
}

TEST_CASE("Empty Graph")
{
    Graph empty_graph({});
    MST empty_mst(empty_graph, "prim");

    CHECK(empty_mst.getWieght() == 0);
    CHECK(empty_mst.shortestPath(0, 1).empty());
    CHECK(empty_mst.longestPath(0, 1).empty());
    CHECK(empty_mst.averageDist() == -1);
}

/*
class comlexTestGraph
{
public:
    static Graph createSampleGraph()
    {
        vector<vector<int>> adj_matrix = {
            {0, 2, 0, 6, 0},
            {2, 0, 3, 8, 5},
            {0, 3, 0, 0, 7},
            {6, 8, 0, 0, 9},
            {0, 5, 7, 9, 0}};
        return Graph(adj_matrix);
    }

    static Graph createLargeGraph(int size, int maxWeight)
    {
        vector<vector<int>> adj_matrix(size, vector<int>(size, 0));
        std::mt19937 gen(std::chrono::system_clock::now().time_since_epoch().count());
        std::uniform_int_distribution<> dis(1, maxWeight);

        for (int i = 0; i < size; ++i)
        {
            for (int j = i + 1; j < size; ++j)
            {
                if (dis(gen) % 3 == 0)
                { // 1/3 chance of edge existing
                    int weight = dis(gen);
                    adj_matrix[i][j] = weight;
                    adj_matrix[j][i] = weight;
                }
            }
        }
        return Graph(adj_matrix);
    }
};
bool isValidMST(Graph &original, const vector<vector<int>> &mst)
{
    int n = original.getAdjMat().size();
    vector<int> parent(n);
    for (int i = 0; i < n; i++)
        parent[i] = i;

    function<int(int)> find = [&](int x)
    {
        if (parent[x] != x)
            parent[x] = find(parent[x]);
        return parent[x];
    };

    function<void(int, int)> unite = [&](int x, int y)
    {
        int px = find(x), py = find(y);
        if (px != py)
            parent[py] = px;
    };

    int mstWeight = 0;
    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            if (mst[i][j] > 0)
            {
                mstWeight += mst[i][j];
                unite(i, j);
            }
        }
    }

    // Check if the MST is connected
    int root = find(0);
    for (int i = 1; i < n; i++)
    {
        if (find(i) != root)
            return false;
    }

    // Check if it's minimal
    int totalWeight = 0;
    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            if (original.getAdjMat()[i][j] > 0)
            {
                totalWeight += original.getAdjMat()[i][j];
            }
        }
    }

    return mstWeight <= totalWeight;
}

TEST_CASE("Complex MST Tests")
{
    SUBCASE("Verify all algorithms produce valid MSTs")
    {
        Graph graph = comlexTestGraph::createLargeGraph(100, 1000);
        vector<string> algorithms = {"prim", "kruskal", "boruvka", "integerMST"};

        for (const auto &algo : algorithms)
        {
            MST mst(graph, algo);
            CHECK(isValidMST(graph, mst.getMST()));
        }
    }

    SUBCASE("Test MST on fully connected graph")
    {
        int n = 20;
        vector<vector<int>> adj_matrix(n, vector<int>(n, 1));
        for (int i = 0; i < n; ++i)
            adj_matrix[i][i] = 0;

        Graph graph(adj_matrix);
        MST mst(graph, "kruskal");

        CHECK(mst.getWieght() == n - 1);
        CHECK(isValidMST(graph, mst.getMST()));
    }

    SUBCASE("Test MST on graph with large weight disparity")
    {
        Graph graph = comlexTestGraph::createLargeGraph(50, 1000000);
        MST mst(graph, "prim");
        CHECK(isValidMST(graph, mst.getMST()));
    }

    SUBCASE("Test path finding on complex graph")
    {
        Graph graph = comlexTestGraph::createLargeGraph(100, 100);
        MST mst(graph, "boruvka");

        vector<int> shortPath = mst.shortestPath(0, 99);
        vector<int> longPath = mst.longestPath(0, 99);

        CHECK(shortPath.front() == 0);
        CHECK(shortPath.back() == 99);
        CHECK(longPath.front() == 0);
        CHECK(longPath.back() == 99);
        CHECK(longPath.size() >= shortPath.size());
    }

    SUBCASE("Test MST algorithms performance")
    {
        Graph graph = comlexTestGraph::createLargeGraph(1000, 10000);
        vector<string> algorithms = {"prim", "kruskal", "boruvka", "integerMST"};

        for (const auto &algo : algorithms)
        {
            auto start = std::chrono::high_resolution_clock::now();
            MST mst(graph, algo);
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            CHECK(duration.count() < 5000); // Ensure each algorithm completes in less than 5 seconds
            CHECK(isValidMST(graph, mst.getMST()));
        }
    }
}

TEST_CASE("Edge Cases and Error Handling")
{
    SUBCASE("Test on disconnected graph")
    {
        vector<vector<int>> adj_matrix = {
            {0, 1, 0, 0},
            {1, 0, 0, 0},
            {0, 0, 0, 1},
            {0, 0, 1, 0}};
        Graph graph(adj_matrix);
        MST mst(graph, "kruskal");

        CHECK(mst.getWieght() == 2);
        CHECK_THROWS(mst.shortestPath(0, 2));
    }

    SUBCASE("Test on graph with negative weights")
    {
        vector<vector<int>> adj_matrix = {
            {0, -1, 2},
            {-1, 0, 3},
            {2, 3, 0}};
        Graph graph(adj_matrix);
        CHECK_THROWS(MST(graph, "prim"));
    }

    SUBCASE("Test on graph with self-loops")
    {
        vector<vector<int>> adj_matrix = {
            {1, 2, 3},
            {2, 2, 4},
            {3, 4, 3}};
        Graph graph(adj_matrix);
        MST mst(graph, "kruskal");
        CHECK(mst.getWieght() == 5);
    }
}

TEST_CASE("Correctness of Integer MST Algorithm")
{
    SUBCASE("Test on graph with large integer weights")
    {
        Graph graph = comlexTestGraph::createLargeGraph(100, 1000000000); // weights up to 10^9
        MST mst(graph, "integerMST");
        CHECK(isValidMST(graph, mst.getMST()));
    }
}*/
