#include "convexHullCmd.h"

#include <limits>
#include <vector>
#include <maya/MFloatPointArray.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MGlobal.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItSelectionList.h>
#include <maya/MSelectionList.h>

#include "utils.h"

MStatus convexHullCmd::doIt(const MArgList& args) {
  MStatus status = MS::kSuccess;

  MSelectionList sList;
  MGlobal::getActiveSelectionList(sList);

  MItSelectionList sListIt(sList);
  bool selectedMesh = false;
  
  for (; !sListIt.isDone(); sListIt.next()) {
    MDagPath dagPath;
    sListIt.getDagPath(dagPath);
    
    if (dagPath.node().hasFn(MFn::kMesh)) {
      selectedMesh = true;
      computeHull(dagPath, &status);
    } else if (dagPath.node().hasFn(MFn::kTransform) && dagPath.hasFn(MFn::kMesh)) {
      selectedMesh = true;
      computeHull(dagPath, &status);
    }
  }

  if (sListIt.isDone() && !selectedMesh) {
    displayError("No shape or transform selected");
    return MS::kFailure;
  }

  setResult("convexHullCmd command executed!\n");
  return MS::kSuccess;
}

void *convexHullCmd::creator() {
  return new convexHullCmd;
}

void convexHullCmd::computeHull(MDagPath dagPath, MStatus *status) {
  MItMeshVertex vertexIt(dagPath, MObject::kNullObj, status);
  
  // Check iterator
  if (*status != MS::kSuccess) {
    displayError("Failed to create vertex iterator");
    return;
  }
  if (vertexIt.isDone()) {
    displayError("No vertices in object");
    return;
  }

  // Initialize extremes
  MPoint extremes[3][2];
  int extremesIndices[3][2];
  for (unsigned int i = 0; i < 3; ++i) {
    extremes[i][0][i] = std::numeric_limits<double>::max();
    extremes[i][1][i] = std::numeric_limits<double>::min();
  }
  
  // Find axial extremes
  MPoint point;
  for (; !vertexIt.isDone(); vertexIt.next()) {
    point = vertexIt.position(MSpace::kWorld, status);

    for (unsigned int i = 0; i < 3; ++i) {
      if (point[i] < extremes[i][0][i]) {
        extremes[i][0] = point;
        extremesIndices[i][0] = vertexIt.index();
      }
      if (point[i] > extremes[i][1][i]) {
        extremes[i][1] = point;
        extremesIndices[i][1] = vertexIt.index();
      }
    }
  }

  // Found extremes
  for (unsigned int i = 0; i < 3; ++i) {
    displayInfo(MString("Found minimum ") + i + ": " + toS(extremes[i][0]));
    displayInfo(MString("Found maximum ") + i + ": " + toS(extremes[i][1]));
  }

  std::vector<bool> usedPoints(vertexIt.count(), false);

  // Find longest pair
  MPoint *longestA = nullptr;
  MPoint *longestB = nullptr;
  double longestDistance = -1.0;
  double distance;
  for (unsigned int i = 0; i < 6; ++i) {
    for (unsigned int j = i + 1; j < 6; ++j) {
      MPoint *tempA = &extremes[i / 2][i % 2];
      MPoint *tempB = &extremes[j / 2][j % 2];
      distance = tempA->distanceTo(*tempB);
      if (distance > longestDistance) {
        longestA = tempA;
        longestB = tempB;
        longestDistance = distance;
      }
    }
  }
  displayInfo("Longest pair from " + toS(*longestA) + " to " + toS(*longestB) + " with distance " + longestDistance);

  // Mark used indices
  for (unsigned int i = 0; i < 6; ++i) {
    if (longestA == &extremes[i / 2][i % 2]) {
      usedPoints.at(extremesIndices[i / 2][i % 2]) = true;
    } else if (longestB == &extremes[i / 2][i % 2]) {
      usedPoints.at(extremesIndices[i / 2][i % 2]) = true;
    }
  }

  // Find furthest point from line
  vertexIt.reset();
  longestDistance = -1.0;
  int pointIndex = -1;
  MPoint furthestPoint;
  for (; !vertexIt.isDone(); vertexIt.next()) {
    point = vertexIt.position(MSpace::kWorld, status);
    distance = perpDistance(point, *longestA, *longestB);
    if (distance > longestDistance) {
      pointIndex = vertexIt.index();
      longestDistance = distance;
      furthestPoint = point;
    }
  }
  displayInfo("Found furthest point " + toS(furthestPoint) + " with distance " + longestDistance);

  // Populate vertex array
  MFloatPointArray vertexArray;
  MFloatPoint floatPoint;
  floatPoint.setCast(*longestA);
  vertexArray.append(floatPoint);
  floatPoint.setCast(*longestB);
  vertexArray.append(floatPoint);
  floatPoint.setCast(furthestPoint);
  vertexArray.append(floatPoint);

  // Populate polygon arrays
  MIntArray polygonCounts(1, 3);
  MIntArray polygonConnects;
  polygonConnects.append(0);
  polygonConnects.append(1);
  polygonConnects.append(2);

  // Create mesh
  MFnMeshData meshDataFn;
  MObject meshDataWrapper = meshDataFn.create();
  MFnMesh meshFn;
  MObject transformObj = meshFn.create(3, 1, vertexArray, polygonCounts, polygonConnects, MObject::kNullObj, status);
  if (*status != MS::kSuccess) {
    displayError("Could not create mesh object");
    return;
  }
}

double convexHullCmd::perpDistance(const MPoint &testPt, const MPoint &linePtA, const MPoint &linePtB) {
  const double num = ((testPt - linePtA) ^ (testPt - linePtB)).length();
  const double den = (linePtB - linePtA).length();
  return num / den;
}