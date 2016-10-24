#include "convexHullCmd.h"

#include <limits>
#include <numeric>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnSet.h>
#include <maya/MGlobal.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItSelectionList.h>
#include <maya/MPointArray.h>
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
      createConvexHull(dagPath, &status);
    } else if (dagPath.node().hasFn(MFn::kTransform) && dagPath.hasFn(MFn::kMesh)) {
      selectedMesh = true;
      createConvexHull(dagPath, &status);
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

void convexHullCmd::createConvexHull(MDagPath dagPath, MStatus *status) {
  // Get target's points
  MFnMesh target(dagPath, status);
  if (MZH::hasError(*status, "Failed to get mesh")) return;
  MPointArray targetPoints;
  *status = target.getPoints(targetPoints, MSpace::kWorld);
  if (MZH::hasError(*status, "Failed to get mesh points")) return;
  
  // Check point array
  if (targetPoints.length() < 4) {
    displayError("Not enough vertices in mesh");
    return;
  }

  // Initialize useable point indices array
  std::list<unsigned int> useablePoints(targetPoints.length());
  std::iota(useablePoints.begin(), useablePoints.end(), 0);

  // Initialize extremes
  MPoint extremes[3][2];
  int extremesIndices[3][2];
  for (unsigned int i = 0; i < 3; ++i) {
    extremes[i][0][i] = std::numeric_limits<double>::max();
    extremes[i][1][i] = std::numeric_limits<double>::min();
  }
  
  // Find axial extremes
  for (unsigned int i = 0; i < targetPoints.length(); ++i) {
    const MPoint &point = targetPoints[i];

    for (unsigned int i = 0; i < 3; ++i) {
      if (point[i] < extremes[i][0][i]) {
        extremes[i][0] = point;
        extremesIndices[i][0] = i;
      }
      if (point[i] > extremes[i][1][i]) {
        extremes[i][1] = point;
        extremesIndices[i][1] = i;
      }
    }
  }

  // Found extremes
  for (unsigned int i = 0; i < 3; ++i) {
    displayInfo(MString("Found minimum ") + i + ": " + MZH::toS(extremes[i][0]));
    displayInfo(MString("Found maximum ") + i + ": " + MZH::toS(extremes[i][1]));
  }

  // Find longest pair
  MPoint *longestA = nullptr;
  MPoint *longestB = nullptr;
  double longestDistance = -1.0;
  for (unsigned int i = 0; i < 6; ++i) {
    for (unsigned int j = i + 1; j < 6; ++j) {
      MPoint *tempA = &extremes[i / 2][i % 2];
      MPoint *tempB = &extremes[j / 2][j % 2];
      double distance = tempA->distanceTo(*tempB);
      if (distance > longestDistance) {
        longestA = tempA;
        longestB = tempB;
        longestDistance = distance;
      }
    }
  }
  displayInfo("Longest pair from "
    + MZH::toS(*longestA) + " to " + MZH::toS(*longestB)
    + " with distance " + longestDistance);

  // Mark used indices
  for (unsigned int i = 0; i < 6; ++i) {
    if (longestA == &extremes[i / 2][i % 2]) {
      //usedPoints.at(extremesIndices[i / 2][i % 2]) = true;
    } else if (longestB == &extremes[i / 2][i % 2]) {
      //usedPoints.at(extremesIndices[i / 2][i % 2]) = true;
    }
  }

  // Find furthest point from line
  longestDistance = -1.0;
  int pointIndex = -1;
  MPoint furthestPoint;
  for (unsigned int i = 0; i < targetPoints.length(); ++i) {
    const MPoint &point = targetPoints[i];
    const double distance = perpDistance(point, *longestA, *longestB);
    if (distance > longestDistance) {
      pointIndex = i;
      longestDistance = distance;
      furthestPoint = point;
    }
  }
  displayInfo("Found furthest point " + MZH::toS(furthestPoint) + " with distance " + longestDistance);

  // Populate vertex array
  MPointArray vertexArray;
  vertexArray.append(*longestA);
  vertexArray.append(*longestB);
  vertexArray.append(furthestPoint);

  // Populate polygon arrays
  MIntArray polygonCounts(1, 3);
  MIntArray polygonConnects;
  polygonConnects.append(0);
  polygonConnects.append(1);
  polygonConnects.append(2);

  // Create mesh
  MObject meshDataWrapper = MFnMeshData().create();
  MObject transform = MFnMesh().create(3, 1, vertexArray, polygonCounts, polygonConnects, MObject::kNullObj, status);
  if (MZH::hasError(*status, "Failed to create convex hull mesh")) return;

  // Rename mesh
  *status = dgModifier.renameNode(transform, "convexHull#");
  if (MZH::hasError(*status, "Failed to add convex hull rename to DGModifier")) return;
  *status = dgModifier.doIt();
  if (MZH::hasError(*status, "Failed to rename the convex hull transform node")) return;

  // Set initial shading
  *status = MZH::setShadingGroup(dgModifier, transform, "initialShadingGroup");
  if (MZH::hasError(*status, "Failed to add shading group change to convex hull mesh")) return;
  *status = dgModifier.doIt();
  if (MZH::hasError(*status, "Failed to set the shading group of the convex hull mesh")) return;

  outputDagPaths.append(MDagPath::getAPathTo(transform, status));
  if (MZH::hasError(*status, "Failed to get a DAG path to the convex hull mesh")) return;
}

void convexHullCmd::createBaseHull(const MPointArray &points, std::list<unsigned int> &useablePoints, MStatus *status) {

}

double convexHullCmd::perpDistance(const MPoint &testPt, const MPoint &linePtA, const MPoint &linePtB) {
  const double num = ((testPt - linePtA) ^ (testPt - linePtB)).length();
  const double den = (linePtB - linePtA).length();
  return num / den;
}