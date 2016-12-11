#pragma once

#include <maya/MFloatPoint.h>
#include <maya/MDGModifier.h>
#include <maya/MPoint.h>
#include <maya/MString.h>

namespace MZH {
  template <typename T>
  T clamp(const T &number, const T &lower, const T &upper) {
    return std::max(lower, std::min(number, upper));
  }

  // Creates a locator using the spaceLocator command
  MStatus createLocator(MDGModifier &dgModifier, const MPoint &position, const MString &name, bool relative = true);

  // Returns true if status is not MS::kSuccess.
  // Prints message on error. Only works for MPxCommand.
  bool hasError(const MStatus &status, const char *message);

  // Sets a transform node's mesh to use shading group group.
  MStatus setShadingGroup(MDGModifier &dgModifier, MObject transform, MString group);
  
  // Converts a MPoint to a MFloatPoint
  MFloatPoint toFP(const MPoint &point);
  
  // Returns a human-readable version of point
  MString toS(const MPoint &point);

  // Returns a human-readable version of some standard type
  template <typename T>
  MString toS(T value) {
    MString result;
    result += value;
    return result;
  }
};