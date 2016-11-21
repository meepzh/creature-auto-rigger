#pragma once

#include "QuickHull.h"

class ACD {
public:
  ACD(MItMeshVertex &vertexIt, MStatus *status = nullptr);

  QuickHull &quickHull();

protected:
  QuickHull quickHull_;
};
