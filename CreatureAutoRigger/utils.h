#pragma once

#include <maya/MFloatPoint.h>
#include <maya/MPoint.h>
#include <maya/MPxCommand.h>
#include <maya/MString.h>

namespace MZH {
  bool hasError(const MStatus &status, const char *message) {
    if (status != MS::kSuccess) {
      MPxCommand::displayError(MString(message) + ": " + status.errorString());
      return true;
    }
    return false;
  }

  MFloatPoint toFP(const MPoint &point) {
    MFloatPoint fp;
    fp.setCast(point);
    return fp;
  }

  MString toS(const MPoint &point) {
    MString result = "(";
    result += point[0];
    result += ", ";
    result += point[1];
    result += ", ";
    result += point[2];
    result += ")";
    return result;
  }
};