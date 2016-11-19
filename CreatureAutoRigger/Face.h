#pragma once

#include <list>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <memory>
#include <vector>

#include "HalfEdge.h"

class Face {
public:
  Face();
  ~Face();

  enum Flag { VISIBLE, NONCONVEX, DELETED };

  void checkConsistency();
  void computeCentroid();
  // Computes the normal, area, and number of vertices
  void computeNormal();
  void computeNormal(double minArea);
  void mergeAdjacentFaces(std::shared_ptr<HalfEdge> adjacentEdge, std::vector<Face *> &discardedFaces);
  double pointPlaneDistance(const MPoint &testPt) const;

  std::shared_ptr<HalfEdge> edge();
  std::shared_ptr<HalfEdge> edge(int i);
  void setEdge(std::shared_ptr<HalfEdge> edge);

  double area() const;
  MPoint centroid() const;
  void clearOutside();
  bool hasOutside() const;
  unsigned int numVertices() const;
  std::list<Vertex *>::iterator outside() const;
  void setOutside(std::list<Vertex *>::iterator outside);

  Flag flag;

  static std::unique_ptr<Face> createTriangle(Vertex *v0, Vertex *v1, Vertex *v2, double minArea = 0);

private:
  void checkVertexCount();
  void computeNormalAndCentroid();
  void computeNormalAndCentroid(double minArea);
  Face *connectHalfEdges(std::shared_ptr<HalfEdge> prevEdge, std::shared_ptr<HalfEdge> nextEdge);
  
  double area_;
  MPoint centroid_;
  std::shared_ptr<HalfEdge> edge_;
  bool hasOutside_;
  MVector normal_;
  unsigned int numVertices_;
  double planeOffset_;
  std::list<Vertex *>::iterator outside_;
};