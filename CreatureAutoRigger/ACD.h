#pragma once

#include <set>
#include <unordered_map>
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
  std::vector<Vertex *> &knots();
  double maxConcavity();
  pEdgeMap &projectedEdges();
  QuickHull &quickHull();
  std::vector<Face *> &vertexBridges();

protected:
  void constructor(MItMeshVertex &vertexIt, MStatus *status);
  void getHullVertices();
  void getNeighbors(MItMeshVertex &vertexIt);
  void projectHullEdges();
  void matchPointsToBridge();
  void calculateConcavities();
  void findKnots();
  std::vector<DPVertex> douglasPeucker(std::vector<DPVertex> &vertices);

  double averageConcavity_;
  std::vector<double> concavities_;
  double concavityTolerance_;
  double clusteringThreshold_;
  double douglasPeuckerThreshold_;
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
