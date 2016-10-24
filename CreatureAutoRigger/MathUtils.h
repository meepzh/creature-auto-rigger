#pragma once

#include <maya/MPoint.h>
#include <maya/MVector.h>

namespace MZH {
  // Distance between a test point and a line defined by 2 points
  double pointLineDistance(const MPoint &testPt, const MPoint &linePtA, const MPoint &linePtB);
  
  // Signed distance between a test point and a plane defined by 3 points
  double pointPlaneDistance(const MPoint &testPt, const MPoint &planePtA, const MPoint &planePtB, const MPoint &planePtC);
  
  // Signed distance between a test point and a plane defined by a point and a unit normal
  double pointPlaneDistance(const MPoint &testPt, const MPoint &planePt, const MVector &unitPlaneNormal);

  // Unit normal for a plane defined by 3 points
  MVector unitPlaneNormal(const MPoint &planePtA, const MPoint &planePtB, const MPoint &planePtC);
};
