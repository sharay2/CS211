// graph.h
// Stephen Harayo
//
// Basic graph class using adjacency matrix representation.  Currently
// limited to a graph with at most 100 vertices.
//
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <map>

using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:
  map<VertexT, map<VertexT, WeightT>> adjacencyList;

 public:
  //constructor:
  graph(){}

  graph& operator=(const graph &other){
    if (this != &other) {
      adjacencyList = other.adjacencyList;
    }
    return *this;
  }

  void clear() {
    adjacencyList.clear();
  }

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
    return static_cast<int>(this->adjacencyList.size()); //size of adjacencyList is the same as the number of vertices in the graph
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const { //iterates through each vertex and adds up edges
    int count = 0;
    for (const auto& entry : adjacencyList) {
      count += entry.second.size();
    }
    return count;
  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph if there's room, and if so
  // returns true.  If the graph is full, or the vertex already
  // exists in the graph, then false is returned.
  //
  bool addVertex(VertexT v) {
    //check if the vertex already exists
    if (adjacencyList.count(v) > 0) {
      return false;
    }

    //add the vertex with an empty set of neighbors
    adjacencyList[v] = map<VertexT,WeightT>();

    return true;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist or for some reason the
  // graph is full, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    auto fromIt = adjacencyList.find(from);
    auto toIt = adjacencyList.find(to);

    if (fromIt == adjacencyList.end() || toIt == adjacencyList.end()) {
      // one of the vertices does not exist
      return false;
    }

    //check if the edge already exists
    auto edgeIt = fromIt->second.find(to);

    if (edgeIt != fromIt->second.end()) {
      // edge already exists, update the weight
      edgeIt->second = weight;
    }
    else {
      // edge does not exist, add the new edge
      fromIt->second[to] = weight;
    }
    return true;
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    auto fromIt = adjacencyList.find(from);

    if (fromIt == adjacencyList.end()) {
      return false; // 'from' vertex not found
    }

    auto toIt = fromIt->second.find(to);

    if (toIt == fromIt->second.end()) {
      return false; // edge not found
    }

    // if the edge exists, return the weight
    weight = toIt->second;
    return true;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;

    auto vertexIt = this->adjacencyList.find(v);

    if (adjacencyList.count(v)!=0) {
      //if vertex is found in the adjacency list, add all of its neighbors to a set
      for (const auto& neighbor : vertexIt->second) {
        S.insert(neighbor.first);
      }
    }
    return S; //returns a set of a vertex's neighbors, empty if vertex is not found
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    vector<VertexT> vertices;

    for(auto const &v : this->adjacencyList){
      vertices.push_back(v.first);
    }
    return vertices;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;
    int vertexIndex = 0;
    for (const auto& entry : this->adjacencyList) {
        output << " " << vertexIndex++ << ". " << entry.first << endl;
    }

    output << endl;
    output << "**Edges:" << endl;
    int row = 0;
    for (const auto& entry : this->adjacencyList) {
      VertexT from = entry.first;
      output << " row " << from << ": ";

      for (int i = 0; i < this->NumVertices(); ++i) {
        VertexT to = getVertexByIndex(i);
        WeightT weight;

        if (getWeight(from, to, weight)) {
          output << "(T," << weight << ") ";
        }
        else {
          output << "F ";
        }
      }
      output << endl;
      row++;
    }
    output << "**************************************************" << endl;
  }

// Helper function to get the vertex at a specific index
  VertexT getVertexByIndex(int index) const {
    int vertexIndex = 0;
    for (const auto& entry : this->adjacencyList) {
      if (vertexIndex == index) {
        return entry.first;
      }
      vertexIndex++;
    }
    throw out_of_range("Index out of bounds");
  }
};
