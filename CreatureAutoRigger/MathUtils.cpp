#include "MathUtils.h"

#include "Utils.h"

namespace MZH {
  MPointArray douglasPeucker(MPointArray &points, double epsilon) {
    if (points.length() < 3) return points;
    
    MPointArray results;
    double maxDistanceSquared = 0.0;
    unsigned int maxIndex = -1;

    MPoint start = points[0];
    MPoint end = points[points.length() - 1];

    for (unsigned int i = 1; i < points.length() - 1; ++i) {
      const double distanceSquared = pointSegmentDistanceSquared(points[i], start, end);
      if (distanceSquared <= maxDistanceSquared) continue;
      maxIndex = i;
      maxDistanceSquared = distanceSquared;
    }

    if (maxDistanceSquared > epsilon * epsilon) {
      MPointArray slice1 = slice(points, 0, maxIndex + 1);
      MPointArray slice2 = slice(points, maxIndex, points.length());
      MPointArray results1 = douglasPeucker(slice1, epsilon);
      MPointArray results2 = douglasPeucker(slice2, epsilon);
      copy(results, results1);
      copy(results, results2);
    } else {
      results.append(start);
      results.append(end);
    }

    return results;
  }

  double pointLineDistance(const MPoint &testPt, const MPoint &linePtA, const MPoint &linePtB) {
    const double num = ((testPt - linePtA) ^ (testPt - linePtB)).length();
    const double den = (linePtB - linePtA).length();
    return num / den;
  }

  double pointLineDistanceSquared(const MPoint &testPt, const MPoint &linePtA, const MPoint &linePtB) {
    const double num = squareLength((testPt - linePtA) ^ (testPt - linePtB));
    const double den = squareLength(linePtB - linePtA);
    return num / den;
  }

  double pointSegmentDistance(const MPoint &testPt, const MPoint &pointA, const MPoint &pointB) {
    double lengthSquared = squareLength(pointB - pointA);
    if (lengthSquared == 0.0) return (testPt - pointA).length();

    double t = std::max(0.0, std::min(1.0, (testPt - pointA) * (pointB - pointA) / lengthSquared));
    MPoint projection = pointA + t * (pointB - pointA);
    return (testPt - projection).length();
  }

  double pointSegmentDistanceSquared(const MPoint &testPt, const MPoint &pointA, const MPoint &pointB) {
    double lengthSquared = squareLength(pointB - pointA);
    if (lengthSquared == 0.0) return squareLength(testPt - pointA);

    double t = std::max(0.0, std::min(1.0, (testPt - pointA) * (pointB - pointA) / lengthSquared));
    MPoint projection = pointA + t * (pointB - pointA);
    return squareLength(testPt - projection);
  }

  double pointPlaneDistance(const MPoint &testPt, const MPoint &planePtA, const MPoint &planePtB, const MPoint &planePtC) {
    const MVector unitNormal = unitPlaneNormal(planePtA, planePtB, planePtC);
    return pointPlaneDistance(testPt, planePtA, unitNormal);
  }

  double pointPlaneDistance(const MPoint &testPt, const MPoint &planePt, const MVector &unitPlaneNormal) {
    return unitPlaneNormal * (testPt - planePt);
  }

  double squareLength(const MVector &vector) {
    return vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
  }

  MVector unitPlaneNormal(const MPoint &planePtA, const MPoint &planePtB, const MPoint &planePtC) {
    const MVector normal = (planePtB - planePtA) ^ (planePtC - planePtA);
    return normal.normal();
  }
};
