#pragma once

#include <list>
#include <maya/MArgList.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MDGModifier.h>
#include <maya/MPxCommand.h>

// Computes and creates a mesh representing the convex hull of all selected meshes
class ConvexHullCmd : public MPxCommand {
public:
  // Checks the selection list for valid meshes and performs computeHull on them
  MStatus doIt(const MArgList& args);
  // Returns an instance of convexHullCmd
  static void *creator();

protected:
  // Computes and creates a mesh representing the convex hull of the mesh associated with dagPath
  void createConvexHull(MDagPath dagPath, MStatus *status);

private:
  MDGModifier dgModifier;
  MDagPathArray outputDagPaths;
};