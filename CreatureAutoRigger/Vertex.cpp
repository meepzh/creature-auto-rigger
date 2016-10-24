#include "Vertex.h"

Vertex::Vertex(const MPoint &point)
    : face_(nullptr), point_(point) {
}

Face *Vertex::face() {
  return face_;
}

const MPoint &Vertex::point() const {
  return point_;
}

void Vertex::setFace(Face *face) {
  face_ = face;
}
