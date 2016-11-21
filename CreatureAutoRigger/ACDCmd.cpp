#include "ACDCmd.h"

#include <maya/MGlobal.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItSelectionList.h>

#include "Utils.h"

MStatus ACDCmd::doIt(const MArgList& args) {
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
      runACD(dagPath, &status);
    } else if (dagPath.node().hasFn(MFn::kTransform) && dagPath.hasFn(MFn::kMesh)) {
      selectedMesh = true;
      runACD(dagPath, &status);
    }
  }

  if (sListIt.isDone() && !selectedMesh) {
    displayError("No shape or transform selected");
    return MS::kFailure;
  }

  if (status == MS::kSuccess) setResult("ACDCmd command executed!\n");
  return status;
}

void *ACDCmd::creator() {
  return new ACDCmd;
}

void ACDCmd::runACD(MDagPath dagPath, MStatus *status) {
  MItMeshVertex vertexIt(dagPath, MObject::kNullObj, status);
  if (MZH::hasError(*status, "Failed to get vertex iterator")) return;
  if (vertexIt.isDone()) {
    displayWarning("No vertices in object");
    return;
  }

  displayInfo("Number of vertices: " + MZH::toS(vertexIt.count()));
  for (; !vertexIt.isDone(); vertexIt.next()) {
    displayInfo("Vertex " + MZH::toS(vertexIt.index()) + "/" + MZH::toS(vertexIt.count()));
  }
}
