#include "ACD.h"

#include "Utils.h"

ACD::ACD(MItMeshVertex &vertexIt, MStatus *status)
    : quickHull_(vertexIt, -1, status) {
  if (MZH::hasError(*status, "Error running QuickHull")) return;
  vertexIt.reset();
}

QuickHull &ACD::quickHull() {
  return quickHull_;
}
