#pragma once

#include <list>
#include <maya/MPoint.h>
#include <maya/MVector.h>

#include "HalfEdge.h"
#include "Vertex.h"

class Face {
public:
  Face();

private:
  MPoint centroid_;
  MVector normal_;
  std::list<vertex> verticesOutside_;
};