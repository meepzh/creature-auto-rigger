#pragma once

#include <list>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <memory>

#include "HalfEdge.h"

class Face {
public:
  Face();
  ~Face();

  void computeCentroid();
  // Computes the normal, area, and number of vertices
  void computeNormal();
  void computeNormal(double minArea);
  double pointPlaneDistance(const MPoint &testPt);

  HalfEdge *edge();
  HalfEdge *edge(int i);
  void setEdge(std::shared_ptr<HalfEdge> &edge);

  bool hasOutside() const;
  std::list<Vertex *>::iterator outside() const;
  void setOutside(std::list<Vertex *>::iterator outside);

  static std::unique_ptr<Face> createTriangle(Vertex *v0, Vertex *v1, Vertex *v2, double minArea = 0);

private:
  void computeNormalAndCentroid();
  void computeNormalAndCentroid(double minArea);
  
  double area_;
  MPoint centroid_;
  std::shared_ptr<HalfEdge> edge_;
  bool hasOutside_;
  MVector normal_;
  unsigned int numVertices_;
  double planeOffset_;
  std::list<Vertex *>::iterator outside_;
};