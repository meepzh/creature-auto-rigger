#pragma once

#include <maya/MPoint.h>
#include <maya/MString.h>

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