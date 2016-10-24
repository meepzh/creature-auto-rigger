#pragma once

#include <maya/MFloatPoint.h>
#include <maya/MDGModifier.h>
#include <maya/MPoint.h>
#include <maya/MString.h>

namespace MZH {
  // Craetes a locator using the spaceLocator command
  MStatus createLocator(MDGModifier &dgModifier, const MPoint &position, const MString &name, bool relative = true);

  // Returns true if status is not MS::kSuccess.
  // Prints message on error. Only works for MPxCommand.
  bool hasError(const MStatus &status, const char *message);

  // Sets a transform node's mesh to use shading group group.
  MStatus setShadingGroup(MDGModifier &dgModifier, MObject transform, MString group);
  
  // Converts a MPoint to a MFloatPoint
  MFloatPoint toFP(const MPoint &point);
  
  // Returns a human-readable version of double
  MString toS(double value);
  // Returns a human-readable version of point
  MString toS(const MPoint &point);
};