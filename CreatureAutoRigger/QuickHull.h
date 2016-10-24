#pragma once

#include <list>
#include <maya/MPointArray.h>
#include <vector>

#include "Face.h"

class QuickHull {
public:
  QuickHull(const MPointArray &points, MStatus *status = nullptr);
  
  // Constructs the convex hull
  MStatus build(const MPointArray &points);

protected:
  void buildHull();
  void buildSimplexHull();
  MStatus computeMinMax(Vertex *&v0, Vertex *&v1);
  void initBuffers(unsigned int numPoints);
  void setPoints(const MPointArray &points);

  std::vector<std::unique_ptr<Face>> faces_;
  std::vector<Vertex> pointBuffer_;
  double tolerance_;

private:
  void addVertexToFace(Vertex &vertex, Face *face);
  
  std::list<Vertex> claimed_;
};
