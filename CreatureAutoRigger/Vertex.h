#pragma once

#include <maya/MPoint.h>

class Face;

class Vertex {
public:
  Vertex(const MPoint &point);
  
  const Face *face() const;
  const MPoint &point() const;

  void setFace(const Face *face);

private:
  const Face *face_;
  const MPoint point_;
};
