#include "Vertex.h"

Vertex::Vertex(const MPoint &point, int index)
    : face_(nullptr), point_(point), index_(index) {
}

Face *Vertex::face() {
  return face_;
}

int Vertex::index() const {
  return index_;
}

const MPoint &Vertex::point() const {
  return point_;
}

void Vertex::setFace(Face *face) {
  face_ = face;
}
