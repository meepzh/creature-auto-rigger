#pragma once

#include <set>
#include <unordered_map>
#include <unordered_set>
#include "QuickHull.h"

typedef std::unordered_map<Vertex *, std::unordered_map<Vertex *, std::shared_ptr<std::vector<Vertex *>>>> pEdgeMap;

struct DPVertex {
  DPVertex(const MPoint &point, Vertex *vertex) {
    this->point = point;
    this->vertex = vertex;
  }
  MPoint point;
  Vertex *vertex;
};

class ACD {
public:
  ACD(MItMeshVertex &vertexIt, double concavityTolerance, double douglasPeuckerThreshold, MStatus *status = nullptr);
  ACD(MItMeshVertex &vertexIt, double concavityTolerance, double clusteringThreshold, double douglasPeuckerThreshold, MStatus *status = nullptr);

  double averageConcavity();
  std::vector<double> &concavities();
  std::vector<Vertex *> &hullVertices();
  std::unordered_set<Vertex *> &knots();
  double maxConcavity();
  pEdgeMap &projectedEdges();
  QuickHull &quickHull();
  std::vector<Face *> &vertexBridges();

protected:
  void constructor(MItMeshVertex &vertexIt);
  void getHullVertices();
  void getNeighbors(MItMeshVertex &vertexIt);
  void projectHullEdges();
  void matchPointsToBridge();
  void calculateConcavities();
  void findKnots();
  std::vector<DPVertex> douglasPeucker(std::vector<DPVertex> &vertices);

  double averageConcavity_;
  double concavityTolerance_;
  double clusteringThreshold_;
  double douglasPeuckerThreshold_;
  double maxConcavity_;

  // Concavity by vertex ID
  std::vector<double> concavities_;
  // Vertices of the convex hull
  std::vector<Vertex *> hullVertices_;
  // Map of convex hull vertices to their convex hull vertex neighbors
  std::unordered_map<Vertex *, std::vector<Vertex *>> hullVertexNeighbors_;
  // Set of vertices identified as knots
  std::unordered_set<Vertex *> knots_;
  // Vertex neighbors by vertex ID
  std::vector<std::vector<Vertex *>> neighbors_;
  // Map of (source vertex, (target vertex, vector path from source to target or the opposite))
  pEdgeMap projectedEdges_;
  // Completed QuickHull instance
  QuickHull quickHull_;
  MStatus returnStatus_;
  // Vertex objects by Vertex ID
  std::shared_ptr<std::vector<Vertex>> vertices_;
  // Map of vertices to possible vertex bridges (faces)
  std::unordered_map<Vertex *, std::set<Face *>> vertexBridgeList_;
  // Vertex bridges by vertex ID
  std::vector<Face *> vertexBridges_;
};
