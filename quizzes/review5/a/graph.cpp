#include <set>
#include <queue>
#include "graph.h"
#include <map>

void Graph::addVertex(vertex v) {
  //Insert a vertex with no edges
    weightedGraph_[v];
}

bool Graph::hasVertex(vertex v) {
  //Check if the vertex v, exists in the graph
    if(weightedGraph_.find(v) != weightedGraph_.end()){
        return true;
    }
  return false;
}

void Graph::addEdge(vertex u, vertex v, weight w) {
  //Assumes that u & v have already been added to the graph
  //We need to record the same edge twice as this is an undirected graph
  weightedGraph_.at(u).insert({v, w}); //Inserting an edge between u and v, with weight w
  weightedGraph_.at(v).insert({u, w}); //Inserting an edge between v and u, with weight w
}


std::vector<Graph::vertex> Graph::getVertices(void) {
  //Iterate through the weightedGraph_ and push back each vertex to the vertices vector
    std::vector<vertex> vertices;
    std::map<vertex, edges_t>::iterator i;
    for (i=weightedGraph_.begin(); i != weightedGraph_.end(); i++){
        vertices.push_back(i->first);
    }
  return std::vector<Graph::vertex>();

}

void Graph::getChildren(std::vector<Graph::vertex> *path, Graph::vertex parent){
    Graph::edges_t children=weightedGraph_[parent];
    std::map<Graph::vertex, Graph::weight>::iterator i;
    bool pathChildren=true;
    for(i=children.begin(); i != children.end(); i++){
    vertex a=i->first;
    bool onPath=false;
    for (auto b: *path){
        if(a==b){
            onPath=true;
            continue;
        }
    }
    if(!onPath){
        path->push_back(a);
        pathChildren=false;
    }
}
    if(pathChildren){
        if((*path)[iterate]==path->back()){
            return;
        }
    }
    iterate++;
    vertex next=(*path)[iterate];
    getChildren(path,next);
}


std::vector<Graph::vertex> Graph::bfs(vertex start) {
  //Perform a breadth first search on the entire graph
    iterate=0;
    std::vector<Graph::vertex> path;
    path.push_back(start);
    if(hasVertex(start)){
getChildren(&path, start);
    }
    else{
        std::cout<< "Invalid" << std::endl;
    }
                    return path;
  //return std::vector<Graph::vertex>();
}
