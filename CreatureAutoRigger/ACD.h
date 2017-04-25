#pragma once

#include <maya/MItMeshEdge.h>
#include <maya/MItMeshPolygon.h>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include "QuickHull.h"

struct DPVertex {
  DPVertex(const MPoint &point, Vertex *vertex) {
    this->point = point;
    this->vertex = vertex;
  }
  MPoint point;
  Vertex *vertex;
};

struct PocketCut {
  PocketCut() {
    noncrossing = true;
    weight = 0.0;
  }
  bool noncrossing;
  std::vector<const Vertex *> path;
  double weight;
};

typedef std::unordered_map<Vertex *, std::unordered_map<Vertex *, std::shared_ptr<std::vector<Vertex *>>>> pEdgeMap;
typedef std::unordered_map<const Vertex *, std::unordered_map<const Vertex *, std::shared_ptr<PocketCut>>> pathWeightMap;

class ACD {
public:
  ACD(MItMeshEdge &edgeIt, MItMeshPolygon &faceIt, MItMeshVertex &vertexIt,
    double concavityTolerance, double douglasPeuckerThreshold,
    MStatus *status = nullptr);
  ACD(MItMeshEdge &edgeIt, MItMeshPolygon &faceIt, MItMeshVertex &vertexIt,
    double concavityTolerance, double clusteringThreshold, double douglasPeuckerThreshold,
    MStatus *status = nullptr);

  double averageConcavity();
  std::vector<double> &concavities();
  std::vector<Vertex *> &hullVertices();
  std::unordered_set<Vertex *> &knots();
  double maxConcavity();
  pEdgeMap &projectedEdges();
  QuickHull &quickHull();
  std::vector<Face *> &vertexBridges();

protected:
  void constructor();
  void getHullVertices();
  void getNeighbors();
  void projectHullEdges();
  void matchPointsToBridge();
  void calculateConcavities();
  void findKnots();
  std::vector<DPVertex> douglasPeucker(std::vector<DPVertex> &vertices);
  void computePocketCuts();
  double weighPath(const std::vector<const Vertex *>& path);
  double calculateCurvature(const Vertex *vertexA, const Vertex *vertexB);

  double averageConcavity_;
  double concavityTolerance_;
  double clusteringThreshold_;
  double douglasPeuckerThreshold_;
  double maxConcavity_;

  // Mesh accessors
  MItMeshEdge &edgeIt_;
  MItMeshPolygon &faceIt_;
  MItMeshVertex &vertexIt_;

  // Concavity by vertex ID
  std::vector<double> concavities_;
  std::unordered_map<const Vertex *, std::unordered_map<const Vertex *, double>> curvatures_;
  // Vertices of the convex hull
  std::vector<Vertex *> hullVertices_;
  // Map of convex hull vertices to their convex hull vertex neighbors
  std::unordered_map<Vertex *, std::vector<Vertex *>> hullVertexNeighbors_;
  // Set of vertices identified as knots
  std::unordered_set<Vertex *> knots_;
  // Vertex neighbors by vertex ID
  std::vector<std::vector<Vertex *>> neighbors_;
  pathWeightMap pocketCutMap_;
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
