#pragma once

#include <list>
#include <maya/MIntArray.h>
#include <maya/MPointArray.h>
#include <vector>

#include "Face.h"

class QuickHull {
public:
  QuickHull(const MPointArray &points, MStatus *status = nullptr);
  
  // Constructs the convex hull
  MStatus build(const MPointArray &points);
  void mayaExport(int &numVertices, int &numPolygons, MPointArray &vertexArray, MIntArray &polygonCounts, MIntArray &polygonConnects);

  std::vector<Vertex *> debugVertices;

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
  void addVertexToFace(Vertex *vertex, Face *face);
  
  std::list<Vertex *> claimed_;
};
