CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -pedantic -g
INCLUDES = -I.
LIBS =

# Source files for Pipeline server
PIPELINE_SOURCES = Graph.cpp MST.cpp PipelineServer.cpp
PIPELINE_OBJECTS = $(PIPELINE_SOURCES:.cpp=.o)

# Source files for Leader-Follower server
LEADER_FOLLOWER_SOURCES = Graph.cpp MST.cpp LeaderFollowerServer.cpp
LEADER_FOLLOWER_OBJECTS = $(LEADER_FOLLOWER_SOURCES:.cpp=.o)

# Executables
PIPELINE_EXEC = pipeline_server
LEADER_FOLLOWER_EXEC = leaderfollower_server

.PHONY: all clean run_pipeline run_leaderfollower

# Default build compiles both servers
all: $(PIPELINE_EXEC) $(LEADER_FOLLOWER_EXEC)

# Compile the Pipeline server
$(PIPELINE_EXEC): $(PIPELINE_OBJECTS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)

# Compile the Leader-Follower server
$(LEADER_FOLLOWER_EXEC): $(LEADER_FOLLOWER_OBJECTS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)

# Run the Pipeline server
run_pipeline: $(PIPELINE_EXEC)
	./$(PIPELINE_EXEC)

# Run the Leader-Follower server
run_leaderfollower: $(LEADER_FOLLOWER_EXEC)
	./$(LEADER_FOLLOWER_EXEC)

clean:
	rm -f $(PIPELINE_OBJECTS) $(LEADER_FOLLOWER_OBJECTS) $(PIPELINE_EXEC) $(LEADER_FOLLOWER_EXEC)