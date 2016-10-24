#pragma once

#include <list>
#include <maya/MArgList.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MDGModifier.h>
#include <maya/MPxCommand.h>

// Computes and creates a mesh representing the convex hull of all selected meshes
class convexHullCmd : public MPxCommand {
public:
  // Checks the selection list for valid meshes and performs computeHull on them
  MStatus doIt(const MArgList& args);
  // Returns an instance of convexHullCmd
  static void *creator();

protected:
  // Computes and creates a mesh representing the convex hull of the mesh associated with dagPath
  void createConvexHull(MDagPath dagPath, MStatus *status);
  // Creates the initial tetrahedron from the most extreme points
  void createBaseHull(const MPointArray &points, std::list<unsigned int> &useablePoints, MStatus *status);
  // Returns the shortest distance between testPt and the line represented by linePtA and linePtB
  static double perpDistance(const MPoint &testPt, const MPoint &linePtA, const MPoint &linePtB);

private:
  MDGModifier dgModifier;
  MDagPathArray outputDagPaths;
};