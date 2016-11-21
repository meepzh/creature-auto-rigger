#pragma once

#include <maya/MPoint.h>

class Face;

class Vertex {
public:
  Vertex(const MPoint &point, int index);
  
  Face *face();
  int index() const;
  const MPoint &point() const;

  void setFace(Face *face);

private:
  Face *face_;
  int index_;
  const MPoint point_;
};
