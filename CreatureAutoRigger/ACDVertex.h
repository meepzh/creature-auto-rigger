#pragma once

#include "Vertex.h"

#include <vector>

class ACDVertex : public Vertex {
public:
  ACDVertex(const MPoint &point, int index);

private:
  int index_;
  std::vector<ACDVertex *> neighbors_;
};
