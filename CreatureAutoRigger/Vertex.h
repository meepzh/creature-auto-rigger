#pragma once

#include <maya/MPoint.h>

class Face;

class Vertex {
public:
  Vertex(const MPoint &point);
  
  Face *face();
  const MPoint &point() const;

  void setFace(Face *face);

private:
  Face *face_;
  const MPoint point_;
};
