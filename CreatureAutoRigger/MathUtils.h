#pragma once

#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MVector.h>

namespace MZH {
  // Clamp a value between a min and max number
  template <typename T>
  T clamp(const T &number, const T &lower, const T &upper) {
    return std::max(lower, std::min(number, upper));
  }

  // General Douglas-Peucker for simplifying lines spatially
  MPointArray douglasPeucker(MPointArray &points, double epsilon);

  // Distance between a test point and a line defined by 2 points
  double pointLineDistance(const MPoint &testPt, const MPoint &linePtA, const MPoint &linePtB);
  double pointLineDistanceSquared(const MPoint &testPt, const MPoint &linePtA, const MPoint &linePtB);
  
  // Distance between a test point and a line segment defined by 2 points
  double pointSegmentDistance(const MPoint &testPt, const MPoint &pointA, const MPoint &pointB);
  double pointSegmentDistanceSquared(const MPoint &testPt, const MPoint &pointA, const MPoint &pointB);

  // Signed distance between a test point and a plane defined by 3 points
  double pointPlaneDistance(const MPoint &testPt, const MPoint &planePtA, const MPoint &planePtB, const MPoint &planePtC);
  
  // Signed distance between a test point and a plane defined by a point and a unit normal
  double pointPlaneDistance(const MPoint &testPt, const MPoint &planePt, const MVector &unitPlaneNormal);

  // Square length of a vector
  double squareLength(const MVector &vector);

  // Unit normal for a plane defined by 3 points
  MVector unitPlaneNormal(const MPoint &planePtA, const MPoint &planePtB, const MPoint &planePtC);
};
