#pragma once

#include <list>
#include <maya/MPoint.h>
#include <maya/MVector.h>

#include "HalfEdge.h"

class Face {
public:
  Face();

private:
  MPoint centroid_;
  MVector normal_;
  std::list<Vertex> verticesOutside_;
};