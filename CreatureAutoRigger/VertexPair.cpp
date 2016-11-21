#include "VertexPair.h"

#include <cmath>
#include <limits>

VertexPair::VertexPair(Vertex *first, Vertex *second)
  : first(first), second(second), length_(std::numeric_limits<double>::infinity()) {
}

bool VertexPair::operator<(const VertexPair &vp) {
  return length_ < vp.length();
}

bool VertexPair::operator>(const VertexPair &vp) {
  return length_ > vp.length();
}

double VertexPair::calculateLength() {
  length_ = (first->point() - second->point()).length();
  return length_;
}

double VertexPair::length() const {
  return length_;
}
