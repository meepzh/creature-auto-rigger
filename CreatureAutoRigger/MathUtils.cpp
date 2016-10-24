#include "MathUtils.h"

namespace MZH {

  double pointLineDistance(const MPoint &testPt, const MPoint &linePtA, const MPoint &linePtB) {
    const double num = ((testPt - linePtA) ^ (testPt - linePtB)).length();
    const double den = (linePtB - linePtA).length();
    return num / den;
  }

  double pointPlaneDistance(const MPoint &testPt, const MPoint &planePtA, const MPoint &planePtB, const MPoint &planePtC) {
    const MVector unitNormal = unitPlaneNormal(planePtA, planePtB, planePtC);
    return unitNormal * (testPt - planePtA);
  }

  double pointPlaneDistance(const MPoint &testPt, const MPoint &planePt, const MVector &unitPlaneNormal) {
    return unitPlaneNormal * (testPt - planePt);
  }

  MVector unitPlaneNormal(const MPoint &planePtA, const MPoint &planePtB, const MPoint &planePtC) {
    const MVector normal = (planePtB - planePtA) ^ (planePtC - planePtA);
    return normal.normal();
  }
};
