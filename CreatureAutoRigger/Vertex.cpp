#include "Vertex.h"

Vertex::Vertex(const MPoint &point)
    : point_(point) {
}

const MPoint &Vertex::point() const {
  return point_;
}
