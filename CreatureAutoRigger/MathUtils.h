#pragma once

#include <maya/MPoint.h>
#include <maya/MVector.h>

namespace MZH {
  double pointLineDistance(const MPoint &testPt, const MPoint &linePtA, const MPoint &linePtB);
  double pointPlaneDistance(const MPoint &testPt, const MPoint &planePtA, const MPoint &planePtB, const MPoint &planePtC);
  double pointPlaneDistance(const MPoint &testPt, const MPoint &planePt, const MVector &unitPlaneNormal);
  MVector unitPlaneNormal(const MPoint &planePtA, const MPoint &planePtB, const MPoint &planePtC);
};
