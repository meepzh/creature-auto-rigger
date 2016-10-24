#include "ConvexHullCmd.h"

#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MGlobal.h>
#include <maya/MItSelectionList.h>
#include <maya/MPointArray.h>
#include <maya/MSelectionList.h>

#include "QuickHull.h"
#include "Utils.h"

MStatus ConvexHullCmd::doIt(const MArgList& args) {
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

  if (status == MS::kSuccess) setResult("convexHullCmd command executed!\n");
  return status;
}

void *ConvexHullCmd::creator() {
  return new ConvexHullCmd;
}

void ConvexHullCmd::createConvexHull(MDagPath dagPath, MStatus *status) {
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

  // Generate convex hull
  QuickHull qc(targetPoints);

  // Debug vertices
  unsigned int debugCount = 0;
  for (auto it = qc.debugVertices.begin(); it != qc.debugVertices.end(); ++it) {
    MZH::createLocator(dgModifier, (*it)->point(), MString("v") + debugCount, false);
    ++debugCount;
  }

  // Export from QuickHull
  int numVertices = 0;
  int numPolygons = 0;
  MPointArray vertexArray;
  MIntArray polygonCounts;
  MIntArray polygonConnects;
  qc.mayaExport(numVertices, numPolygons, vertexArray, polygonCounts, polygonConnects);

  // Create mesh
  MObject meshDataWrapper = MFnMeshData().create();
  MObject transform = MFnMesh().create(numVertices, numPolygons, vertexArray, polygonCounts, polygonConnects, MObject::kNullObj, status);
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
