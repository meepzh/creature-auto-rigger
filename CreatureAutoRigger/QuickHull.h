#pragma once

#include <list>
#include <maya/MIntArray.h>
#include <maya/MPointArray.h>
#include <vector>

#include "Face.h"

class QuickHull {
public:
  QuickHull(const MPointArray &points, int maxIterations = -1, MStatus *status = nullptr);
  
  enum MergeType { NONCONVEX, NONCONVEX_WRT_LARGER_FACE };

  // Constructs the convex hull
  MStatus build(const MPointArray &points);
  void mayaExport(int &numVertices, int &numPolygons, MPointArray &vertexArray, MIntArray &polygonCounts, MIntArray &polygonConnects);

  std::vector<Vertex *> debugVertices;

protected:
  void addNewFaces(Vertex *eyeVertex);
  void addVertexToHull(Vertex *eyeVertex);
  void sanityCheck();
  void buildHull();
  void buildSimplexHull();
  void clearDeletedFaces();
  void computeHorizon(const MPoint &point, std::shared_ptr<HalfEdge> crossedEdge, Face *face);
  MStatus computeMinMax(Vertex *&v0, Vertex *&v1);
  void deleteFaceVertices(Face *face, Face *absorbingFace = nullptr);
  void initBuffers(unsigned int numPoints);
  Vertex *nextVertexToAdd();
  double oppositeFaceDistance(HalfEdge *he) const;
  void resolveUnclaimedPoints();
  void setPoints(const MPointArray &points);

  std::vector<std::unique_ptr<Face>> faces_;
  std::vector<std::shared_ptr<HalfEdge>> horizon_;
  int maxIterations_;
  std::vector<Vertex> vertices_;
  double tolerance_;

private:
  void addVertexToFace(Vertex *vertex, Face *face);
  std::shared_ptr<HalfEdge> addAdjoiningFace(Vertex *eyeVertex, std::shared_ptr<HalfEdge> horizonEdge);
  bool doAdjacentMerge(Face *face, MergeType mergeType);
  std::list<Vertex *> removeAllVerticesFromFace(Face *face);
  void removeVertexFromFace(Vertex *vertex, Face *face);

  std::list<Vertex *> claimed_;
  std::list<Face *> newFaces_;
  std::vector<Vertex *> pending_;
  std::list<Vertex *> unclaimed_;
};
