#include "ConvexHullCmd.h"

#include <maya/MArgDatabase.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MGlobal.h>
#include <maya/MItSelectionList.h>
#include <maya/MPointArray.h>
#include <maya/MSelectionList.h>

#include "QuickHull.h"
#include "Utils.h"

#define kIterationsFlagLong "-iterations"
#define kIterationsFlag "-i"

MStatus ConvexHullCmd::doIt(const MArgList& args) {
  MStatus status = MS::kSuccess;
  int maxIterations = -1;

  status = parseArgs(args, maxIterations);
  if (MZH::hasError(status, "Error parsing arguments")) return status;
  displayInfo("Max Iterations: " + MZH::toS(maxIterations));

  MSelectionList sList;
  MGlobal::getActiveSelectionList(sList);

  MItSelectionList sListIt(sList);
  bool selectedMesh = false;
  
  for (; !sListIt.isDone(); sListIt.next()) {
    MDagPath dagPath;
    sListIt.getDagPath(dagPath);
    
    if (dagPath.node().hasFn(MFn::kMesh)) {
      selectedMesh = true;
      createConvexHull(dagPath, maxIterations, &status);
    } else if (dagPath.node().hasFn(MFn::kTransform) && dagPath.hasFn(MFn::kMesh)) {
      selectedMesh = true;
      createConvexHull(dagPath, maxIterations, &status);
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

MSyntax ConvexHullCmd::newSyntax() {
  MSyntax syntax;
  syntax.addFlag(kIterationsFlag, kIterationsFlagLong, MSyntax::kLong);
  return syntax;
}

void ConvexHullCmd::createConvexHull(MDagPath dagPath, int maxIterations, MStatus *status) {
  MItMeshVertex vertexIt(dagPath, MObject::kNullObj, status);
  if (MZH::hasError(*status, "Failed to get vertex iterator")) return;
  
  // Check point array
  if (vertexIt.count() < 4) {
    displayError("Not enough vertices in mesh");
    return;
  }

  // Generate convex hull
  QuickHull qc(vertexIt, maxIterations, status);
  if (MZH::hasError(*status, "Error running QuickHull")) return;

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

MStatus ConvexHullCmd::parseArgs(const MArgList &args, int &maxIterations) {
  MStatus status = MS::kSuccess;

  MArgDatabase argData(syntax(), args);
  if (argData.isFlagSet(kIterationsFlag, &status)) {
    status = argData.getFlagArgument(kIterationsFlag, 0, maxIterations);
  }

  return status;
}
