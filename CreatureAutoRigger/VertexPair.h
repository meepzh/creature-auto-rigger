#pragma once

#include "Vertex.h"

class VertexPair {
public:
  VertexPair(Vertex *first, Vertex *second);

  bool operator<(const VertexPair &vp);
  bool operator>(const VertexPair &vp);

  double calculateLength();
  double length() const;

  Vertex *first;
  Vertex *second;

protected:
  double length_;
};
