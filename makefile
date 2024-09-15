CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -pedantic -g
INCLUDES = -I.
LIBS =

# Source files
SOURCES = Graph.cpp MST.cpp
TEST_SOURCES = MST_test.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)
TEST_OBJECTS = $(TEST_SOURCES:.cpp=.o)

# Executables
TEST = test_mst

.PHONY: all clean test

all: $(TEST)

$(TEST): $(OBJECTS) $(TEST_OBJECTS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

test: $(TEST)
	./$(TEST)

clean:
	rm -f $(OBJECTS) $(TEST_OBJECTS) $(TEST)