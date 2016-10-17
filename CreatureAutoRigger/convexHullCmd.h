#pragma once

#include <maya/MArgList.h>
#include <maya/MDagPath.h>
#include <maya/MPxCommand.h>

class convexHullCmd : public MPxCommand {
public:
  MStatus doIt(const MArgList& args);
  static void *creator();

protected:
  void computeHull(MDagPath dagPath, MStatus *status);
  static double perpDistance(const MPoint &testPt, const MPoint &linePtA, const MPoint &linePtB);
};