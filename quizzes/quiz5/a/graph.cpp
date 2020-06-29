#include <set>
#include <queue>
#include "graph.h"
#include <iostream>

void Graph::addVertex(vertex v)
{
  //Insert a vertex with no edges
  weightedGraph_.insert({v, edges_t()});
  //weightedGraph_.insert({v,edges_t(v,0)};
}

bool Graph::hasVertex(vertex v)
{
  //Check if the vertex v, exists in the graph
  if (weightedGraph_.find(v) != weightedGraph_.end())
  {
    return true;
  }
  return false;
}

void Graph::addEdge(vertex u, vertex v, weight w)
{
  //Assumes that u & v have already been added to the graph
  //We need to record the same edge twice as this is an undirected graph
  weightedGraph_.at(u).insert({v, w}); //Inserting an edge between u and v, with weight w
  weightedGraph_.at(v).insert({u, w}); //Inserting an edge between v and u, with weight w
}

std::vector<Graph::vertex> Graph::getVertices(void)
{
  //Iterate through the weightedGraph_ and push back each vertex to the vertices vector
  std::vector<Graph::vertex> data;
  for (auto itr = weightedGraph_.begin(); itr != weightedGraph_.end(); itr++)
  {
    //std::cout << itr->first;
    //std::vector<Graph::vertex>().push_back(itr->first);
    data.push_back(itr->first);
  }

  //return std::vector<Graph::vertex>();
  return data;
}

std::vector<Graph::vertex> Graph::bfs(vertex start)
{
  //Perform a breadth first search on the entire graph
  std::vector<Graph::vertex> data;
  std::queue<vertex> q;
  std::set<vertex> visited;
  vertex node;
  q.push(start);
  visited.insert(start);

  while (!q.empty())
  {
    node = q.front();

    for (auto it = weightedGraph_.at(node).begin(); it != weightedGraph_.at(node).end(); it++)
    {
      if (visited.find(it->first) == visited.end())
      {
        visited.insert(it->first);
        q.push(it->first);
      }
    }

    data.push_back(q.front());
    q.pop();
  }
  return data;
}
