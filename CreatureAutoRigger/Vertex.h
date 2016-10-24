#pragma once

#include <maya/MPoint.h>

class Vertex {
public:
  Vertex(const MPoint &point);
  const MPoint &point() const;

private:
  MPoint point_;
};
