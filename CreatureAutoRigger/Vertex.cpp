#include "Vertex.h"

Vertex::Vertex(const MPoint &point)
    : face_(nullptr), point_(point) {
}

const Face *Vertex::face() const {
  return face_;
}

const MPoint &Vertex::point() const {
  return point_;
}

void Vertex::setFace(const Face *face) {
  face_ = face;
}
