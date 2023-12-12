// application.cpp
// Stephen Harayo
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
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <algorithm>
#include <queue>
#include <sstream>
#include <stack>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"


using namespace std;
using namespace tinyxml2;

const double INF = numeric_limits<double>::max();

//finds the building's information based on the name that the user has given us
BuildingInfo searchBuilding(string query, vector<BuildingInfo>& Buildings){
  BuildingInfo temp; //default value
  for(const auto &e : Buildings){
    if(query == e.Abbrev){ return e;} //checks for abbreviations first and returns the building's info
    if(e.Fullname.find(query)!= string::npos){ return e;} //checks if it is part of the full name of the building and returns the building's info
  }
  return temp; //returns temp, which is empty, if building is not found
}

//looks for the building closest to the center coordinates
BuildingInfo searchCenter(vector<BuildingInfo>& Buildings, Coordinates centerCoords){
  double min = INF;
  BuildingInfo center;
  for(const auto &building : Buildings){
    double distance = distBetween2Points(centerCoords.Lat, centerCoords.Lon, building.Coords.Lat, building.Coords.Lon);
    if(distance<min){
      min = distance;
      center = building;
    }
  }
  return center;
}

//finds the nearest node to a specific building and returns the coordings of the walkway
Coordinates nearestNode(graph<long long, double>& G, const BuildingInfo& targetBuilding, const map<long long, Coordinates>& Nodes, const vector<FootwayInfo>& Footways) {
  // Initialize the minimum distance to a large value
  double minDist = INF;
  
  // Initialize the coordinates of the nearest node
  Coordinates nearestCoords;

  // iterate through each footway in the vector Footways
  for (const auto& footway : Footways) {
    // get the vector of node IDs that make up the footway
    const vector<long long>& nodes = footway.Nodes;

    //iterate through each node ID in the footway
    for (size_t i = 0; i < nodes.size(); ++i) {
      //get the current node ID
      long long nodeID = nodes[i];

      //check if the nodeID is in the Nodes map
      auto nodeIter = Nodes.find(nodeID);
      if (nodeIter != Nodes.end()) {
        //get the coordinates of the current node
        Coordinates nodeCoords = nodeIter->second;

        //calculate the distance between the target building and the current node
        double distance = distBetween2Points(targetBuilding.Coords.Lat, targetBuilding.Coords.Lon, nodeCoords.Lat, nodeCoords.Lon);

        //check if the current node is closer than the previously found nearest node
        if (distance < minDist) {
        //update the minimum distance and set the nearest coordinates to the current node's coordinates
          minDist = distance;
          nearestCoords = nodeCoords;
        }
      }
    }
  }

  //return the coordinates of the nearest node
  return nearestCoords;
}

//prints out the shortest pathway by popping the stack
void printShortestPath(stack<long long>& shortestPath) {
  cout << "Path: ";
  while (!shortestPath.empty()) {
    cout << shortestPath.top();
    shortestPath.pop();
    if (!shortestPath.empty())
    cout << "->";
  }
}

//Dijkstra's algorithm for finding the shortest path
template <typename VertexT, typename WeightT>
vector<VertexT> dijkstra(graph<VertexT, WeightT>& G, VertexT startV, map<VertexT, WeightT>& distances, map<VertexT, VertexT>& pred){
  vector<VertexT> visited; //vector to store the order of visited vertices
  set<VertexT> visitedSet; //set to keep track of already visited vertices
  priority_queue<pair<WeightT, VertexT>, vector<pair<WeightT, VertexT>>, greater<pair<WeightT, VertexT>>> unvisitedQueue;

  //initiliazing the distances and predecessors for all vertices into the unvisitedQueue
  for (const VertexT& vertex : G.getVertices()) {
    unvisitedQueue.emplace(INF, vertex);
    distances[vertex] = INF;
    pred[vertex] = VertexT();
  }
  //set distance at the start as 0 and add it to the priority queue
  unvisitedQueue.emplace(0, startV);
  distances[startV] = 0;

  //Dijkstra's algorithm
  while (!unvisitedQueue.empty()) {
    auto [currentDist, currentVertex] = unvisitedQueue.top();
    unvisitedQueue.pop();

    //if the vertex is INF, then there are no other vertices it can go to
    if (currentDist == INF)
      break;

    //if a vertex is already visited, then skip it
    if (!visitedSet.insert(currentVertex).second)
      continue;

    //adds vertex to the visited vector
    visited.push_back(currentVertex);

    //checks the neighbors of specified vertex
    for (const VertexT& neighbor : G.neighbors(currentVertex)) {
      WeightT edgeWeight = WeightT();
      G.getWeight(currentVertex, neighbor, edgeWeight);
      WeightT altPathDistance = currentDist + edgeWeight;

      //if a shorter path is found, then update the distance and predecessor
      if (altPathDistance < distances[neighbor]) {
        unvisitedQueue.emplace(altPathDistance, neighbor);
        distances[neighbor] = altPathDistance;
        pred[neighbor] = currentVertex;
      }
    }
  }
  return visited; //returns the order of visited vertices
}

// helper function that iterates through the map of paths and adds it to a stack to read
template <typename VertexT>
stack<VertexT> getPath(map<VertexT, VertexT>& predecessors, VertexT begin, VertexT end) {
    stack<VertexT> s;

    if (begin == end) { //if person 1 and 2 are the same, then there is no path
        s.push(begin);
        return s;
    }
    //traverse through the map backwards and add it to a stack
    for (VertexT current = end; current != begin; current = predecessors[current]) {
        s.push(current);
    }
    //push the starting vertex on the stack
    s.push(begin);

    return s;
}
//
// Implement your standard application here
//
void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
  string person1Building, person2Building;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);
    cout << endl;

    //searches for buildings 1 and 2
    BuildingInfo building1 = searchBuilding(person1Building, Buildings);
    BuildingInfo building2 = searchBuilding(person2Building, Buildings);

    if(building1.Fullname == "") //checks if building1 isn't found
      cout << "Person 1's building not found" << endl;
    else if(building2.Fullname == "") //checks if building2 isn't found
      cout << "Person 2's building not found" << endl;
    else{ //if both buildings are found, calculate the distances, nearest nodes, and the shortest pathways using Dijkstra's algorithm
      //prints the information of building1
      cout << "Person 1's point:" << endl;
      cout << " " << building1.Fullname << endl;
      cout << " (" << building1.Coords.Lat << ", " << building1.Coords.Lon << ")" << endl;

      //prints the information of building2
      cout << "Person 2's point:" << endl;
      cout << " " << building2.Fullname << endl;
      cout << " (" << building2.Coords.Lat << ", " << building2.Coords.Lon << ")" << endl;

      //finding the midpoint, then searching for the nearest node to the coordinates of the midpoint
      Coordinates midpoint = centerBetween2Points(building1.Coords.Lat, building1.Coords.Lon, building2.Coords.Lat, building2.Coords.Lon);
      BuildingInfo centerBuilding = searchCenter(Buildings, midpoint);

      //destination is the building nearest to the center coordinates
      cout << "Destination Building:" << endl;
      cout << " " << centerBuilding.Fullname << endl;
      cout << " (" << centerBuilding.Coords.Lat << ", " << centerBuilding.Coords.Lon << ")" << endl << endl;

      //finding the nearest node to building 1, and printing its information
      Coordinates nearP1 = nearestNode(G, building1, Nodes, Footways);
      cout << "Nearest P1 node:" << endl;
      cout << " " << nearP1.ID << endl;
      cout << " (" << nearP1.Lat << ", " << nearP1.Lon << ")" << endl;

      //finding the nearest node to building 2, and printing its information
      Coordinates nearP2 = nearestNode(G, building2, Nodes, Footways);
      cout << "Nearest P2 node:" << endl;
      cout << " " << nearP2.ID << endl;
      cout << " (" << nearP2.Lat << ", " << nearP2.Lon << ")" << endl;

      //finding the nearest node to the center building, and printing its information
      Coordinates nearCenter = nearestNode(G, centerBuilding, Nodes, Footways);
      cout << "Nearest destination node:" << endl;
      cout << " " << nearCenter.ID << endl;
      cout << " (" << nearCenter.Lat << ", " << nearCenter.Lon << ")" << endl;

      //finding the shortest path to the center building for both person 1 and 2
      map<long long, double> distances1, distances2; //
      map<long long, long long> pred1, pred2;

      //running Dijkstra's algorithm for both person 1 and 2
      vector<long long> visited1 = dijkstra(G, nearP1.ID, distances1, pred1);
      vector<long long> visited2 = dijkstra(G, nearP2.ID, distances2, pred2);

      //if there is no path, then error meessage prints
      if (distances2[nearCenter.ID] == INF || distances1[nearCenter.ID] == INF) {
          cout << "Sorry, destination unreachable." << endl;
      }
      else {
        //if paths exist for both people, convert pred1 and pred2 into a stack that allows information to be read
        auto path1 = getPath(pred1, nearP1.ID, nearCenter.ID);
        auto path2 = getPath(pred2, nearP2.ID, nearCenter.ID);

        //printing person 1's distance and path
        cout << "Person 1's distance to dest: " << distances1[nearCenter.ID] << " miles" << endl;
        printShortestPath(path1);
        cout << endl;

        //printing person 2's distance and path
        cout << "Person 2's distance to dest: " << distances2[nearCenter.ID] << " miles" << endl;
        printShortestPath(path2);
        cout << endl;
      }
    }

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }    
}

int main() {
  graph<long long, double> G;

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;

  //graph<long long, double> G;

  // TODO MILESTONE 5: add vertices
  for(const auto &node : Nodes){
    G.addVertex(node.first);
  }
  // TODO MILESTONE 6: add edges
 for (const auto& footway : Footways) {
    const vector<long long>& nodes = footway.Nodes;

    for (size_t i = 0; i < nodes.size() - 1; ++i) {
        long long fNode = nodes[i];
        long long nNode = nodes[i + 1];

        // the distance between nodes using distBetween2Points
        double distance = distBetween2Points(Nodes[fNode].Lat, Nodes[fNode].Lon,Nodes[nNode].Lat, Nodes[nNode].Lon);

        // bidirectional edges with the weight equal to the distance
        G.addEdge(fNode, nNode, distance);
        G.addEdge(nNode, fNode, distance);
    }
}
  

  // TODO: uncomment below after MILESTONE 6
  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
