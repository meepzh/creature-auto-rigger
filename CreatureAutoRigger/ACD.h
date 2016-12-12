#pragma once

#include <set>
#include <unordered_map>
#include "QuickHull.h"

typedef std::unordered_map<Vertex *, std::unordered_map<Vertex *, std::shared_ptr<std::vector<Vertex *>>>> pEdgeMap;

class ACD {
public:
  ACD(MItMeshVertex &vertexIt, MStatus *status = nullptr);

  double averageConcavity();
  std::vector<double> &concavities();
  std::vector<Vertex *> &hullVertices();
  double maxConcavity();
  pEdgeMap &projectedEdges();
  QuickHull &quickHull();
  std::vector<Face *> &vertexBridges();

protected:
  void getHullVertices();
  void getNeighbors(MItMeshVertex &vertexIt);
  void projectHullEdges();
  void matchPointsToBridge();
  void calculateConcavities();
  void findKnots();

  double averageConcavity_;
  std::vector<double> concavities_;
  std::vector<Vertex *> hullVertices_;
  std::unordered_map<Vertex *, std::vector<Vertex *>> hullNeighbors_;
  std::vector<Vertex *> knots_;
  double maxConcavity_;
  std::vector<std::vector<Vertex *>> neighbors_;
  pEdgeMap projectedEdges_;
  QuickHull quickHull_;
  MStatus returnStatus_;
  std::shared_ptr<std::vector<Vertex>> vertices_;
  std::unordered_map<Vertex *, std::set<Face *>> vertexBridgeList_;
  std::vector<Face *> vertexBridges_;
};
