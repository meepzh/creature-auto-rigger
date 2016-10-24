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
  void computeMinMax(Vertex *&v0, Vertex *&v1);
  void initBuffers(unsigned int numPoints);
  void setPoints(const MPointArray &points);

  std::vector<Face> faces_;
  std::vector<Vertex> pointBuffer_;
  double tolerance_;

private:
  
};
