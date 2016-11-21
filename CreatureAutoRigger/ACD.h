#pragma once

#include <unordered_map>
#include "QuickHull.h"

typedef std::unordered_map<Vertex *, std::unordered_map<Vertex *, std::shared_ptr<std::vector<Vertex *>>>> pEdgeMap;

class ACD {
public:
  ACD(MItMeshVertex &vertexIt, MStatus *status = nullptr);

  std::vector<Vertex *> &hullVertices();
  pEdgeMap &projectedEdges();
  QuickHull &quickHull();

protected:
  void getHullVertices();
  void getNeighbors(MItMeshVertex &vertexIt);
  void projectHullEdges();

  std::vector<Vertex *> hullVertices_;
  std::unordered_map<Vertex *, std::vector<Vertex *>> hullNeighbors_;
  std::vector<std::vector<Vertex *>> neighbors_;
  pEdgeMap projectedEdges_;
  QuickHull quickHull_;
  std::shared_ptr<std::vector<Vertex>> vertices_;
};
