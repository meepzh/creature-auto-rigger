#pragma once

#include <list>
#include <maya/MIntArray.h>
#include <maya/MPointArray.h>
#include <vector>

#include "Face.h"

class QuickHull {
public:
  QuickHull(const MPointArray &points, MStatus *status = nullptr);
  
  enum MergeType { NONCONVEX, NONCONVEX_WRT_LARGER_FACE };

  // Constructs the convex hull
  MStatus build(const MPointArray &points);
  void mayaExport(int &numVertices, int &numPolygons, MPointArray &vertexArray, MIntArray &polygonCounts, MIntArray &polygonConnects);

  std::vector<Vertex *> debugVertices;

protected:
  void addNewFaces(Vertex *eyeVertex);
  void addVertexToHull(Vertex *eyeVertex);
  void buildHull();
  void buildSimplexHull();
  void computeHorizon(const MPoint &point, HalfEdge *crossedEdge, Face *face);
  MStatus computeMinMax(Vertex *&v0, Vertex *&v1);
  void deleteFaceVertices(Face *face, Face *absorbingFace = nullptr);
  void initBuffers(unsigned int numPoints);
  Vertex *nextVertexToAdd();
  double oppositeFaceDistance(HalfEdge *he) const;
  void resolveUnclaimedPoints();
  void setPoints(const MPointArray &points);

  std::vector<std::unique_ptr<Face>> faces_;
  std::vector<HalfEdge *> horizon_;
  std::vector<Vertex> vertices_;
  double tolerance_;

private:
  void addVertexToFace(Vertex *vertex, Face *face);
  HalfEdge *addAdjoiningFace(Vertex *eyeVertex, HalfEdge *he);
  bool doAdjacentMerge(Face *face, MergeType mergeType);
  std::list<Vertex *> removeAllVerticesFromFace(Face *face);
  void removeVertexFromFace(Vertex *vertex, Face *face);

  std::list<Vertex *> claimed_;
  std::list<Face *> newFaces_;
  std::vector<Vertex *> pending_;
  std::list<Vertex *> unclaimed_;
};
